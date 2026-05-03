#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 内录文件相机，发布图像与录制姿态数据
constructor_args:
  - runtime:
      file_path: "capture.avi"
      imu_csv_path: "capture_imu.csv"
      camera_name: "camera"
      image_topic_name: "camera_image"
      imu_topic_name: "camera_imu"
      realtime: true
      loop: false
template_args:
  - Info:
      width: 1440
      height: 1080
      step: 4320
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [-0.091821039187099038, 0.46399073468302049, 0.0026098786426372819, 0.0009819586010405485, -0.47512788503104569]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 0.0, 1.0, 0.0]
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "capture_file_package.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"

/**
 * @class CaptureFileCamera
 * @brief 文件相机源，用干净视频和同帧 IMU CSV 按 CameraBase 合约发布数据。
 *
 * 模块只消费清洗后的 capture package：视频文件只包含图像，IMU CSV 显式
 * 给出每帧的 timestamp/quat/gyro/accl。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CaptureFileCamera : public LibXR::Application,
                          public CameraBase<CameraInfoV>
{
 public:
  using Self = CaptureFileCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using ImageFrame = typename Base::ImageFrame;
  using ImuVector = std::array<float, 3>;
  using ImuSample = CaptureFileCameraDetail::ImuSample;
  using QuatSample = std::array<float, 4>;

  // 编译期相机模型来自 BSP YAML 预设。
  static inline constexpr auto camera_info = Base::camera_info;

  // 视频图像当前按紧密排列的 BGR8 处理。
  static constexpr int channel_count = 3;
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  static constexpr int frame_height = static_cast<int>(camera_info.height);

  static constexpr uint32_t image_sink_wait_log_ms = 1000;

  // OpenCV 解码和图像拷贝不是嵌入式小栈任务，线程栈显式放大。
  static constexpr std::size_t capture_thread_stack_bytes = 256 * 1024;

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "CaptureFileCamera currently publishes BGR8 frames");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "CaptureFileCamera expects tightly packed BGR8 frames");

  /**
   * @struct RuntimeParam
   * @brief xrobot YAML 传入的运行时参数。
   */
  struct RuntimeParam
  {
    std::string_view file_path = "capture.avi";  ///< 干净视频路径。
    std::string_view imu_csv_path = "capture_imu.csv";  ///< 同帧 IMU CSV。
    std::string_view camera_name = "camera";  ///< CameraBase 相机名，也是原始 IMU 话题前缀。
    std::string_view image_topic_name = "camera_image";  ///< 图像话题，供 CameraFrameSync 消费。
    std::string_view imu_topic_name = "camera_imu";  ///< 同步后 IMU 话题名。
    bool realtime = true;  ///< 是否按录制帧间隔限速播放。
    bool loop = false;  ///< EOF 后是否回到第 0 帧继续播放。
    uint32_t max_frames = 0;  ///< 0 表示不限帧数，测试可用环境变量覆盖。
  };

  // 构造阶段先检查文件和分辨率，再启动后台采集线程。
  explicit CaptureFileCamera(LibXR::HardwareContainer& hw,
                             LibXR::ApplicationManager& app,
                             RuntimeParam runtime)
      : Base(hw, runtime.camera_name, runtime.image_topic_name, runtime.imu_topic_name),
        file_path_(runtime.file_path),
        imu_csv_path_(runtime.imu_csv_path),
        runtime_(runtime),
        gyro_topic_name_(this->NameView(), "_gyro"),
        accl_topic_name_(this->NameView(), "_accl"),
        quat_topic_name_(this->NameView(), "_quat"),
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<ImuVector>(gyro_topic_name_.CStr())),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<ImuVector>(accl_topic_name_.CStr())),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatSample>(quat_topic_name_.CStr()))
  {
    ApplyEnvironmentOverrides();
    ValidateVideo();
    LoadImuCsv();
    running_.store(true);
    capture_thread_.Create(this, CaptureThreadMain, "CaptureFileCam",
                           capture_thread_stack_bytes, LibXR::Thread::Priority::HIGH);
    app.Register(*this);
  }

  // 通知采集线程在下一轮循环退出。
  ~CaptureFileCamera() override { running_.store(false); }

  void OnMonitor() override
  {
    XR_LOG_INFO("CaptureFileCamera monitor: frames=%u imu=%u running=%d",
                frames_committed_.load(), imu_published_.load(), running_.load() ? 1 : 0);
  }

  void SetExposure(double) override {}

  void SetGain(double) override {}

 private:
  uint64_t FramePeriodUsForIndex(std::size_t index) const
  {
    if (index > 0 && index < imu_samples_.size() &&
        imu_samples_[index].timestamp_us > imu_samples_[index - 1].timestamp_us)
    {
      return imu_samples_[index].timestamp_us - imu_samples_[index - 1].timestamp_us;
    }
    return video_info_.fallback_period_us;
  }

  // 校验解码后的图像是否满足 CameraInfo 编译期约束。
  static bool FrameShapeMatches(const cv::Mat& bgr)
  {
    return bgr.cols == frame_width && bgr.rows == frame_height &&
           bgr.elemSize() == channel_count;
  }

  // 环境变量只用于测试限帧/加速，正常配置仍以 YAML 为准。
  void ApplyEnvironmentOverrides()
  {
    if (const char* env = std::getenv("CAPTURE_FILE_CAMERA_MAX_FRAMES"))
    {
      const unsigned long parsed = std::strtoul(env, nullptr, 10);
      if (parsed > 0UL)
      {
        runtime_.max_frames = static_cast<uint32_t>(parsed);
      }
    }
    if (const char* env = std::getenv("CAPTURE_FILE_CAMERA_REALTIME"))
    {
      runtime_.realtime = !(env[0] == '0' && env[1] == '\0');
    }
  }

  // 构造期预打开一次文件，提前发现路径和分辨率错误。
  void ValidateVideo()
  {
    cv::VideoCapture cap(file_path_);
    if (!cap.isOpened())
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open '%s'", file_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open file");
    }

    video_info_.width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    video_info_.height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    video_info_.fps = cap.get(cv::CAP_PROP_FPS);
    if (video_info_.fps > 0.0 && std::isfinite(video_info_.fps))
    {
      video_info_.fallback_period_us = static_cast<uint64_t>(
          std::llround(CaptureFileCameraDetail::microseconds_per_second / video_info_.fps));
    }

    if (video_info_.width != frame_width || video_info_.height != frame_height)
    {
      XR_LOG_ERROR("CaptureFileCamera geometry mismatch: file=%dx%d constexpr=%dx%d",
                   video_info_.width, video_info_.height, frame_width, frame_height);
      throw std::runtime_error("CaptureFileCamera: geometry mismatch");
    }

    const auto fps_milli = static_cast<unsigned>(std::llround(video_info_.fps * 1000.0));
    const auto period_us = static_cast<unsigned>(video_info_.fallback_period_us);
    XR_LOG_PASS("CaptureFileCamera opened %s: %dx%d fps_milli=%u period_us=%u",
                file_path_.c_str(), video_info_.width, video_info_.height, fps_milli,
                period_us);
  }

  // 构造期加载显式 IMU 文件，运行时只按帧索引取样。
  void LoadImuCsv()
  {
    std::ifstream input(imu_csv_path_);
    if (!input.is_open())
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open IMU csv '%s'",
                   imu_csv_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open imu csv");
    }

    std::string line;
    uint32_t line_number = 0;
    bool header_skipped = false;
    while (std::getline(input, line))
    {
      ++line_number;
      if (CaptureFileCameraDetail::IsSkippableCsvLine(line))
      {
        continue;
      }

      ImuSample sample{};
      if (!CaptureFileCameraDetail::ParseImuCsvRow(line, sample))
      {
        if (imu_samples_.empty() && !header_skipped)
        {
          header_skipped = true;
          continue;
        }
        XR_LOG_ERROR("CaptureFileCamera invalid IMU csv line %u", line_number);
        throw std::runtime_error("CaptureFileCamera: invalid imu csv line");
      }
      imu_samples_.push_back(sample);
    }

    if (imu_samples_.empty())
    {
      XR_LOG_ERROR("CaptureFileCamera IMU csv is empty: '%s'", imu_csv_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: empty imu csv");
    }

    XR_LOG_PASS("CaptureFileCamera loaded %u IMU samples from %s",
                static_cast<unsigned>(imu_samples_.size()), imu_csv_path_.c_str());
  }

  // 等 CameraFrameSync 接好图像 sink，避免启动阶段白丢前几帧。
  void WaitForImageSink()
  {
    uint32_t waited_ms = 0;
    while (running_.load() && !this->ImageSinkReady())
    {
      LibXR::Thread::Sleep(1);
      ++waited_ms;
      if (waited_ms % image_sink_wait_log_ms == 0)
      {
        XR_LOG_WARN("CaptureFileCamera waiting image sink: %u ms", waited_ms);
      }
    }
  }

  // 先发布原始 IMU，再提交图像。采样时间戳走 Topic envelope，
  // 由同步模块负责与图像帧对齐。
  void PublishRawImu(const ImuSample& sample)
  {
    ImuVector gyro_msg = sample.gyro_xyz;
    ImuVector accl_msg = sample.accl_xyz;
    QuatSample quat_msg = sample.quat_wxyz;
    const LibXR::MicrosecondTimestamp timestamp(sample.timestamp_us);

    raw_gyro_topic_.Publish(gyro_msg, timestamp);
    raw_accl_topic_.Publish(accl_msg, timestamp);
    raw_quat_topic_.Publish(quat_msg, timestamp);
    imu_published_.fetch_add(1);
  }

  // 把 BGR 图像写入 CameraBase 租借区并提交。
  bool WriteAndCommitImage(const cv::Mat& bgr, uint64_t timestamp_us)
  {
    if (!this->ImageSinkReady())
    {
      return false;
    }

    ImageFrame* image = this->GetWritableImage();
    if (image == nullptr)
    {
      XR_LOG_WARN("CaptureFileCamera writable image is null");
      return false;
    }

    image->timestamp_us = timestamp_us;
    for (int row = 0; row < frame_height; ++row)
    {
      std::memcpy(image->data.data() + static_cast<std::size_t>(row) * frame_step,
                  bgr.ptr(row), frame_step);
    }

    if (!this->CommitImage())
    {
      XR_LOG_WARN("CaptureFileCamera image commit failed");
      return false;
    }

    frames_committed_.fetch_add(1);
    return true;
  }

  // 单帧处理：读显式 IMU、提交图像、按需限速。
  bool ProcessFrame(const cv::Mat& decoded, const ImuSample& imu, uint64_t period_us)
  {
    cv::Mat bgr;
    if (!CaptureFileCameraDetail::ConvertToBgr(decoded, bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera unsupported cv type=%d", decoded.type());
      return false;
    }
    if (!FrameShapeMatches(bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera decoded frame shape mismatch");
      return false;
    }

    PublishRawImu(imu);
    const bool committed = WriteAndCommitImage(bgr, imu.timestamp_us);

    if (runtime_.realtime)
    {
      const uint64_t sleep_ms =
          std::max<uint64_t>(1U,
                             period_us /
                                 CaptureFileCameraDetail::microseconds_per_millisecond);
      LibXR::Thread::Sleep(static_cast<uint32_t>(sleep_ms));
    }

    return committed;
  }

  // 采集线程主循环；loop 模式下视频和 IMU 索引一起回到起点。
  void CaptureLoop()
  {
    WaitForImageSink();
    cv::VideoCapture cap(file_path_);
    if (!cap.isOpened())
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open '%s' in worker", file_path_.c_str());
      running_.store(false);
      return;
    }

    cv::Mat decoded;
    std::size_t frame_index = 0;
    while (running_.load())
    {
      if (frame_index >= imu_samples_.size())
      {
        XR_LOG_PASS("CaptureFileCamera reached IMU EOF after %u committed frames",
                    frames_committed_.load());
        if (!runtime_.loop)
        {
          running_.store(false);
          break;
        }
        cap.set(cv::CAP_PROP_POS_FRAMES, 0.0);
        frame_index = 0;
        continue;
      }

      if (!cap.read(decoded))
      {
        XR_LOG_PASS("CaptureFileCamera reached EOF after %u committed frames",
                    frames_committed_.load());
        if (!runtime_.loop)
        {
          running_.store(false);
          break;
        }
        cap.set(cv::CAP_PROP_POS_FRAMES, 0.0);
        frame_index = 0;
        continue;
      }

      const uint64_t period_us = FramePeriodUsForIndex(frame_index);
      (void)ProcessFrame(decoded, imu_samples_[frame_index], period_us);
      ++frame_index;
      if (runtime_.max_frames != 0U && frames_committed_.load() >= runtime_.max_frames)
      {
        XR_LOG_PASS("CaptureFileCamera reached max_frames=%u", runtime_.max_frames);
        running_.store(false);
        break;
      }
    }
  }

  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  std::string file_path_{};  ///< RuntimeParam 是 string_view，这里持有路径副本。
  std::string imu_csv_path_{};  ///< 显式 IMU CSV 路径副本。

  RuntimeParam runtime_{};  ///< 应用环境变量覆盖后的运行时参数。
  std::vector<ImuSample> imu_samples_{};  ///< 与视频帧按索引对应的 IMU 数据。

  LibXR::RuntimeStringView<> gyro_topic_name_{};  ///< `<camera_name>_gyro`。
  LibXR::RuntimeStringView<> accl_topic_name_{};  ///< `<camera_name>_accl`。
  LibXR::RuntimeStringView<> quat_topic_name_{};  ///< `<camera_name>_quat`。

  LibXR::Topic raw_gyro_topic_{};  ///< CameraFrameSync 消费的原始陀螺话题。
  LibXR::Topic raw_accl_topic_{};  ///< CameraFrameSync 消费的原始加速度话题。
  LibXR::Topic raw_quat_topic_{};  ///< CameraFrameSync 消费的原始姿态话题。

  LibXR::Thread capture_thread_{};  ///< 解码和发布线程。

  std::atomic<bool> running_{false};  ///< 采集线程退出标志。

  std::atomic<uint32_t> frames_committed_{0};  ///< 已提交图像帧数。

  std::atomic<uint32_t> imu_published_{0};  ///< 已发布原始 IMU 组数。

  CaptureFileCameraDetail::VideoInfo video_info_{};  ///< 视频几何和限速回退周期。
};
