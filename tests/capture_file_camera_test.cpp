#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "CaptureFileCamera.hpp"
#include "libxr.hpp"

namespace
{
namespace fs = std::filesystem;

void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

constexpr CameraTypes::CameraCalibration CALIBRATION{
    1440,
    1080,
    2328.69,
    2328.67,
    733.36,
    540.62,
    {-0.0918, 0.4640, 0.0026, 0.0010, -0.4751}};

/// 写一段录像：第 i 帧时间戳 1000 + 20000·i，像素首字节为 i，奇数帧为 NARROW。
/// Write a recording: frame i at 1000 + 20000·i, first pixel byte i, odd frames NARROW.
std::string WriteRecording(const char* name, int frames, bool with_imu)
{
  const fs::path dir = fs::temp_directory_path() / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  std::ofstream csv(dir / "frames.csv");
  csv << "frame,timestamp_us,frame_counter,roi_x,roi_y,decimation"
      << (with_imu ? ",qw,qx,qy,qz,gx,gy,gz,ax,ay,az" : "") << "\n";
  std::string pixels(CameraTypes::FRAME_BYTES, '\0');
  for (int i = 0; i < frames; ++i)
  {
    const bool narrow = i % 2 == 1;
    csv << i << "," << 1000 + 20000 * i << "," << 10 + i << "," << (narrow ? 400 : 80)
        << "," << (narrow ? 284 : 24) << "," << (narrow ? 1 : 2);
    if (with_imu)
    {
      csv << ",1,0,0,0," << i << ",0,0,0,0,9.8";
    }
    csv << "\n";
    pixels[0] = static_cast<char>(i);
    char file[32];
    std::snprintf(file, sizeof(file), "%06d.pgm", i);
    std::ofstream(dir / file, std::ios::binary) << "P5\n640 512\n255\n" << pixels;
  }
  return dir.string();
}

/// 按到达顺序记录图像与同步帧 / Records images and synced frames in arrival order.
struct Events
{
  struct Event
  {
    char kind;  // 'i' image, 's' synced
    uint64_t timestamp_us;
    const ImageFrame* image;
    float first_pixel;
    CameraTypes::FrameGeometry geometry;
    uint32_t counter;
    uint64_t sequence;
    AutoAim::ImuSample imu;
  };

  std::mutex mutex;
  std::vector<Event> events;
  std::atomic<int> synced{0};

  explicit Events(const std::string& camera)
  {
    auto on_image = LibXR::Topic::Callback::Create(
        [](bool, Events* self, ImageTopicPayload payload)
        {
          const ImageFrame& f = **payload;
          self->Add({'i',
                     static_cast<uint64_t>(f.timestamp_us),
                     &f,
                     static_cast<float>(f.data[0]),
                     f.geometry,
                     f.frame_counter,
                     0,
                     {}});
        },
        this);
    auto on_synced = LibXR::Topic::Callback::Create(
        [](bool, Events* self, const AutoAim::SyncedFrame* s)
        {
          const ImageFrame& f = *s->image;
          self->Add({'s', static_cast<uint64_t>(f.timestamp_us), &f,
                     static_cast<float>(f.data[0]), f.geometry, f.frame_counter,
                     s->sequence, s->imu});
          self->synced.fetch_add(1);
        },
        this);
    LibXR::Topic::CreateTopic<ImageTopicPayload>(StageTopicName(camera, "image").c_str())
        .RegisterCallback(on_image);
    LibXR::Topic::CreateTopic<const AutoAim::SyncedFrame*>(
        StageTopicName(camera, AutoAim::STAGE_SYNCED).c_str())
        .RegisterCallback(on_synced);
  }

  void Add(const Event& e)
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(e);
  }

  std::vector<Event> Synced()
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Event> out;
    for (const Event& e : events)
    {
      if (e.kind == 's') out.push_back(e);
    }
    return out;
  }

  bool WaitSynced(int count, int timeout_ms = 3000)
  {
    for (int t = 0; t < timeout_ms && synced.load() < count; ++t)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return synced.load() >= count;
  }
};

/// 在相机之前建好 Topic 并订阅，收到全部帧。LibXR 回调注册后不能注销，订阅者在堆上
/// 创建且不释放。
/// Create the Topics and subscribe before the camera so every frame arrives. LibXR
/// callbacks cannot be unregistered, so subscribers live on the heap forever.
Events& Subscribe(const char* camera) { return *new Events(camera); }

void TestOrderAndContent()
{
  const std::string dir = WriteRecording("cfc_camera_order", 3, true);
  Events& events = Subscribe("order");
  CaptureFileCamera camera(CALIBRATION, "order", {dir, 0.0, false, 0});
  Expect(events.WaitSynced(3), "all frames replayed");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lock(events.mutex);
  Expect(events.events.size() == 6, "image and synced frame for each of 3 frames");
  for (int n = 0; n < 3; ++n)
  {
    const Events::Event& image = events.events[2 * n];
    const Events::Event& synced = events.events[2 * n + 1];
    Expect(image.kind == 'i' && synced.kind == 's', "image precedes its synced frame");
    Expect(synced.image == image.image, "synced frame holds the published image");
    const uint64_t t = static_cast<uint64_t>(1000 + 20000 * n);
    Expect(image.timestamp_us == t, "recorded timestamp");
    Expect(static_cast<uint64_t>(synced.imu.timestamp_us) == t, "IMU carries image time");
    Expect(synced.imu.angular_velocity_xyz[0] == static_cast<float>(n) &&
               synced.imu.linear_acceleration_xyz[2] == 9.8F,
           "recorded IMU");
    Expect(synced.sequence == static_cast<uint64_t>(n + 1), "sequence increases");
    Expect(image.first_pixel == static_cast<float>(n), "pixels of frame n");
    Expect(image.counter == static_cast<uint32_t>(10 + n), "frame counter");
    const CameraTypes::FrameGeometry expected =
        n % 2 == 1 ? CameraTypes::FrameGeometry{400, 284, 1} : CameraBase::WIDE_GEOMETRY;
    Expect(image.geometry == expected, "geometry follows the recording");
  }
}

void TestLoopAndMaxFrames()
{
  const std::string dir = WriteRecording("cfc_camera_loop", 3, false);
  Events& events = Subscribe("loop");
  CaptureFileCamera camera(CALIBRATION, "loop", {dir, 0.0, true, 7});
  Expect(events.WaitSynced(7), "seven frames over three passes");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto synced = events.Synced();
  Expect(synced.size() == 7, "max_frames stops the replay");
  for (std::size_t i = 1; i < synced.size(); ++i)
  {
    Expect(synced[i].timestamp_us > synced[i - 1].timestamp_us,
           "loop keeps time increasing");
  }
  // 一轮 = 首尾 40 ms + 平均间隔 20 ms / One pass = 40 ms first-to-last + 20 ms period.
  Expect(synced[3].timestamp_us == synced[0].timestamp_us + 60000, "pass length");
  for (const Events::Event& e : synced)
  {
    Expect(e.imu.rotation_wxyz[0] == 1.0F && e.imu.linear_acceleration_xyz[2] == 9.80665F,
           "resting attitude without recorded IMU");
  }
}

void TestPacing()
{
  const std::string dir = WriteRecording("cfc_camera_pace", 4, true);
  Events& events = Subscribe("pace");
  const auto start = std::chrono::steady_clock::now();
  CaptureFileCamera camera(CALIBRATION, "pace", {dir, 1.0, false, 0});
  Expect(events.WaitSynced(4), "paced frames arrive");
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
          .count();
  // 4 帧间隔 20 ms，按录制速度至少 60 ms / Four frames 20 ms apart take >= 60 ms.
  Expect(ms >= 60.0, "paced at the recorded rate");
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  TestOrderAndContent();
  TestLoopAndMaxFrames();
  TestPacing();
  std::puts("capture_file_camera_test passed");
  return 0;
}
