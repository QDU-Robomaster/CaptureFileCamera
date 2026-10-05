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

/// 按到达顺序记录 IMU 与图像 / Records IMU and images in arrival order.
struct Events
{
  struct Event
  {
    char kind;  // 'g' gyro, 'a' accl, 'i' image
    uint64_t timestamp_us;
    float value;  // gyro x / accl z / first pixel byte
    CameraTypes::FrameGeometry geometry;
    uint32_t counter;
  };

  std::mutex mutex;
  std::vector<Event> events;
  std::atomic<int> images{0};

  Events(const std::string& camera, const std::string& gyro, const std::string& accl)
  {
    auto on_gyro = LibXR::Topic::Callback::Create(
        [](bool, Events* self, LibXR::MicrosecondTimestamp t,
           CaptureFileCamera::ImuVector& v)
        { self->Add({'g', static_cast<uint64_t>(t), v.x(), {}, 0}); },
        this);
    auto on_accl = LibXR::Topic::Callback::Create(
        [](bool, Events* self, LibXR::MicrosecondTimestamp t,
           CaptureFileCamera::ImuVector& v)
        { self->Add({'a', static_cast<uint64_t>(t), v.z(), {}, 0}); },
        this);
    auto on_image = LibXR::Topic::Callback::Create(
        [](bool, Events* self, ImageTopicPayload payload)
        {
          const ImageFrame& f = **payload;
          self->Add({'i', static_cast<uint64_t>(f.timestamp_us),
                     static_cast<float>(f.data[0]), f.geometry, f.frame_counter});
          self->images.fetch_add(1);
        },
        this);
    LibXR::Topic::CreateTopic<CaptureFileCamera::ImuVector>(gyro.c_str())
        .RegisterCallback(on_gyro);
    LibXR::Topic::CreateTopic<CaptureFileCamera::ImuVector>(accl.c_str())
        .RegisterCallback(on_accl);
    LibXR::Topic::CreateTopic<ImageTopicPayload>(StageTopicName(camera, "image").c_str())
        .RegisterCallback(on_image);
  }

  void Add(const Event& e)
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(e);
  }

  std::vector<Event> Images()
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Event> out;
    for (const Event& e : events)
    {
      if (e.kind == 'i') out.push_back(e);
    }
    return out;
  }

  bool WaitImages(int count, int timeout_ms = 3000)
  {
    for (int t = 0; t < timeout_ms && images.load() < count; ++t)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return images.load() >= count;
  }
};

/// 在相机之前建好 Topic 并订阅，收到全部帧。LibXR 回调注册后不能注销，订阅者在堆上
/// 创建且不释放。
/// Create the Topics and subscribe before the camera so every frame arrives. LibXR
/// callbacks cannot be unregistered, so subscribers live on the heap forever.
Events& Subscribe(const char* camera, const char* gyro, const char* accl)
{
  return *new Events(camera, gyro, accl);
}

void TestOrderAndGeometry()
{
  const std::string dir = WriteRecording("cfc_camera_order", 3, true);
  Events& events = Subscribe("order", "order_gyro", "order_accl");
  CaptureFileCamera camera(
      CALIBRATION, "order",
      {dir, "order_gyro", "order_accl", "order_quat", 0.0, false, 0});
  Expect(events.WaitImages(3), "all frames replayed");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lock(events.mutex);
  Expect(events.events.size() == 9, "gyro, accl and image for each of 3 frames");
  for (int n = 0; n < 3; ++n)
  {
    const Events::Event& gyro = events.events[3 * n];
    const Events::Event& accl = events.events[3 * n + 1];
    const Events::Event& image = events.events[3 * n + 2];
    Expect(gyro.kind == 'g' && accl.kind == 'a' && image.kind == 'i',
           "IMU precedes image");
    const uint64_t t = static_cast<uint64_t>(1000 + 20000 * n);
    Expect(gyro.timestamp_us == t && image.timestamp_us == t, "recorded timestamps");
    Expect(gyro.value == static_cast<float>(n) && accl.value == 9.8F, "recorded IMU");
    Expect(image.value == static_cast<float>(n), "pixels of frame n");
    Expect(image.counter == static_cast<uint32_t>(10 + n), "frame counter");
    const CameraTypes::FrameGeometry expected =
        n % 2 == 1 ? CameraTypes::FrameGeometry{400, 284, 1} : CameraBase::WIDE_GEOMETRY;
    Expect(image.geometry == expected, "geometry follows the recording");
  }
}

void TestLoopAndMaxFrames()
{
  const std::string dir = WriteRecording("cfc_camera_loop", 3, false);
  Events& events = Subscribe("loop", "loop_gyro", "loop_accl");
  CaptureFileCamera camera(CALIBRATION, "loop",
                           {dir, "loop_gyro", "loop_accl", "loop_quat", 0.0, true, 7});
  Expect(events.WaitImages(7), "seven frames over three passes");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto images = events.Images();
  Expect(images.size() == 7, "max_frames stops the replay");
  for (std::size_t i = 1; i < images.size(); ++i)
  {
    Expect(images[i].timestamp_us > images[i - 1].timestamp_us,
           "loop keeps time increasing");
  }
  // 一轮 = 首尾 40 ms + 平均间隔 20 ms / One pass = 40 ms first-to-last + 20 ms period.
  Expect(images[3].timestamp_us == images[0].timestamp_us + 60000, "pass length");
  std::lock_guard<std::mutex> lock(events.mutex);
  for (const Events::Event& e : events.events)
  {
    if (e.kind == 'a') Expect(e.value == 9.80665F, "resting accl without recorded IMU");
  }
}

void TestPacing()
{
  const std::string dir = WriteRecording("cfc_camera_pace", 4, true);
  Events& events = Subscribe("pace", "pace_gyro", "pace_accl");
  const auto start = std::chrono::steady_clock::now();
  CaptureFileCamera camera(CALIBRATION, "pace",
                           {dir, "pace_gyro", "pace_accl", "pace_quat", 1.0, false, 0});
  Expect(events.WaitImages(4), "paced frames arrive");
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
  TestOrderAndGeometry();
  TestLoopAndMaxFrames();
  TestPacing();
  std::puts("capture_file_camera_test passed");
  return 0;
}
