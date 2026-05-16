#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 文件相机，回放视频图像和 IMU CSV
constructor_args:
  - runtime:
      file_path: "/home/xiao/data/camera_internal_recording_20260428/damo_clean.avi"
      imu_csv_path: "/home/xiao/data/camera_internal_recording_20260428/damo_imu.csv"
      camera_name: "camera"
      image_topic_name: "camera_image"
      imu_topic_name: "camera_imu"
      realtime: true
      loop: false
      max_frames: 0
      replay_speed: 1.0
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
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"

/**
 * @class CaptureFileCamera
 * @brief 文件相机，从视频和同序号 IMU CSV 回放数据。
 *
 * 视频文件提供图像，IMU CSV 提供每帧的 timestamp、quat、gyro 和 accl。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CaptureFileCamera : public LibXR::Application,
                          public CameraBase<CameraInfoV>
{
 public:
  using Self = CaptureFileCamera<CameraInfoV>;  ///< 当前模板实例类型。
  using Base = CameraBase<CameraInfoV>;  ///< CameraBase 基类类型。
  using ImageFrame = typename Base::ImageFrame;  ///< 图像槽数据类型。
  using GyroStamped = typename Base::GyroStamped;  ///< gyro topic 数据类型。
  using AcclStamped = typename Base::AcclStamped;  ///< accl topic 数据类型。
  using QuatStamped = typename Base::QuatStamped;  ///< quat topic 数据类型。

  /// 编译期相机信息。
  static inline constexpr auto camera_info = Base::camera_info;

  /// BGR8 图像通道数。
  static constexpr int channel_count = 3;
  /// 每行字节数。
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  /// 图像宽度。
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  /// 图像高度。
  static constexpr int frame_height = static_cast<int>(camera_info.height);

  /// 缺少可用帧间隔时使用的默认间隔，单位微秒。
  static constexpr uint64_t default_period_us = 10000;
  /// 一秒对应的微秒数。
  static constexpr double microseconds_per_second = 1000000.0;
  /// 一毫秒对应的微秒数。
  static constexpr uint64_t microseconds_per_millisecond = 1000;

  /// OpenCV 解码和图像拷贝线程使用的栈大小。
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
    std::string_view file_path =
        "/home/xiao/data/camera_internal_recording_20260428/damo_clean.avi";  ///< 干净视频路径。
    std::string_view imu_csv_path =
        "/home/xiao/data/camera_internal_recording_20260428/damo_imu.csv";  ///< 同帧 IMU CSV。
    std::string_view camera_name = "camera";  ///< 相机实例名，也是原始 IMU topic 前缀。
    std::string_view image_topic_name = "camera_image";  ///< 图像名称。
    std::string_view imu_topic_name = "camera_imu";  ///< 同步 IMU 名称。
    bool realtime = true;  ///< 是否按录制帧间隔限速播放。
    bool loop = false;  ///< 文件结束后是否回到第 0 帧继续播放。
    uint32_t max_frames = 0;  ///< 0 表示不限帧数，测试可用环境变量覆盖。
    double replay_speed = 1.0;  ///< 回放速度倍率；1.0 表示原速。
  };

  /**
   * @struct ImuSample
   * @brief 与视频帧一一对应的显式 IMU 采样。
   */
  struct ImuSample
  {
    uint64_t timestamp_us{};  ///< 采样时间戳，单位微秒。
    std::array<float, 4> quat_wxyz{};  ///< 姿态四元数。
    std::array<float, 3> gyro_xyz{};  ///< 角速度。
    std::array<float, 3> accl_xyz{};  ///< 线加速度。
  };

  /**
   * @brief 检查输入文件并启动回放线程。
   *
   * @param hw 硬件容器，传给 `CameraBase`。
   * @param app 应用管理器。
   * @param runtime 运行参数。
   *
   * 视频或 CSV 无法打开、格式错误、尺寸不匹配时会抛出 `std::runtime_error`。
   */
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
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<GyroStamped>(gyro_topic_name_.CStr())),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<AcclStamped>(accl_topic_name_.CStr())),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatStamped>(quat_topic_name_.CStr()))
  {
    ApplyEnvironmentOverrides();
    ValidateVideo();
    LoadImuCsv();
    running_.store(true);
    capture_thread_.Create(this, CaptureThreadMain, "CaptureFileCam",
                           capture_thread_stack_bytes, LibXR::Thread::Priority::HIGH);
    app.Register(*this);
  }

  /**
   * @brief 通知回放线程退出。
   */
  ~CaptureFileCamera() override { running_.store(false); }

  /**
   * @brief 打印已提交图像帧数、已发布 IMU 组数和运行状态。
   */
  void OnMonitor() override
  {
    XR_LOG_INFO("CaptureFileCamera monitor: frames=%u imu=%u running=%d",
                frames_committed_.load(), imu_published_.load(), running_.load() ? 1 : 0);
  }

  /**
   * @brief 文件相机不支持曝光设置。
   */
  void SetExposure(double) override {}

  /**
   * @brief 文件相机不支持增益设置。
   */
  void SetGain(double) override {}

 private:
  /**
   * @brief 解析有限浮点数。
   */
  static bool ParseNumber(const std::string& token, double& value)
  {
    const char* begin = token.c_str();
    char* end = nullptr;
    value = std::strtod(begin, &end);
    if (end == begin)
    {
      return false;
    }
    while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end)) != 0)
    {
      ++end;
    }
    return *end == '\0' && std::isfinite(value);
  }

  /**
   * @brief 解析一行 IMU CSV。
   *
   * 列顺序为 `timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az`。
   */
  static bool ParseImuCsvLine(const std::string& line, ImuSample& sample)
  {
    std::array<double, 11> values{};
    std::stringstream stream(line);
    std::string token;
    std::size_t index = 0;
    while (std::getline(stream, token, ','))
    {
      if (index >= values.size() || !ParseNumber(token, values[index]))
      {
        return false;
      }
      ++index;
    }
    if (index != values.size() || values[0] < 0.0)
    {
      return false;
    }

    sample.timestamp_us = static_cast<uint64_t>(std::llround(values[0]));
    sample.quat_wxyz = {static_cast<float>(values[1]), static_cast<float>(values[2]),
                        static_cast<float>(values[3]), static_cast<float>(values[4])};
    sample.gyro_xyz = {static_cast<float>(values[5]), static_cast<float>(values[6]),
                       static_cast<float>(values[7])};
    sample.accl_xyz = {static_cast<float>(values[8]), static_cast<float>(values[9]),
                       static_cast<float>(values[10])};
    return true;
  }

  /**
   * @brief 计算当前帧之后的等待时间，单位微秒。
   */
  uint64_t FramePeriodUsForIndex(std::size_t index) const
  {
    if (index > 0 && index < imu_samples_.size() &&
        imu_samples_[index].timestamp_us > imu_samples_[index - 1].timestamp_us)
    {
      return imu_samples_[index].timestamp_us - imu_samples_[index - 1].timestamp_us;
    }
    return fallback_period_us_;
  }

  /**
   * @brief 检查解码后的图像尺寸和像素字节数。
   */
  static bool FrameShapeMatches(const cv::Mat& bgr)
  {
    return bgr.cols == frame_width && bgr.rows == frame_height &&
           bgr.elemSize() == channel_count;
  }

  /**
   * @brief 应用测试用环境变量。
   */
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
    if (const char* env = std::getenv("CAPTURE_FILE_CAMERA_REPLAY_SPEED"))
    {
      double parsed = 0.0;
      if (ParseNumber(env, parsed) && parsed > 0.0)
      {
        runtime_.replay_speed = parsed;
      }
      else
      {
        XR_LOG_WARN("CaptureFileCamera ignored invalid replay speed '%s'", env);
      }
    }
  }

  /**
   * @brief 打开视频文件并检查宽高。
   */
  void ValidateVideo()
  {
    cv::VideoCapture cap(file_path_);
    if (!cap.isOpened())
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open '%s'", file_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open file");
    }

    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps > 0.0 && std::isfinite(fps))
    {
      fallback_period_us_ =
          static_cast<uint64_t>(std::llround(microseconds_per_second / fps));
    }

    if (width != frame_width || height != frame_height)
    {
      XR_LOG_ERROR("CaptureFileCamera geometry mismatch: file=%dx%d constexpr=%dx%d",
                   width, height, frame_width, frame_height);
      throw std::runtime_error("CaptureFileCamera: geometry mismatch");
    }

    const auto fps_milli = static_cast<unsigned>(std::llround(fps * 1000.0));
    const auto period_us = static_cast<unsigned>(fallback_period_us_);
    const auto speed_milli =
        static_cast<unsigned>(std::llround(runtime_.replay_speed * 1000.0));
    XR_LOG_PASS(
        "CaptureFileCamera opened %s: %dx%d fps_milli=%u period_us=%u realtime=%u replay_speed_milli=%u",
        file_path_.c_str(), width, height, fps_milli, period_us,
        runtime_.realtime ? 1U : 0U, speed_milli);
  }

  /**
   * @brief 加载 IMU CSV。
   */
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
    while (std::getline(input, line))
    {
      ++line_number;
      if (line.empty() || line[0] == '#')
      {
        continue;
      }

      ImuSample sample{};
      if (!ParseImuCsvLine(line, sample))
      {
        if (imu_samples_.empty())
        {
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

  /**
   * @brief 等待图像槽可用。
   */
  void WaitForImageSink()
  {
    while (running_.load() && !this->ImageSinkReady())
    {
      LibXR::Thread::Sleep(1);
    }
  }

  /**
   * @brief 把 OpenCV 解码结果转换成 BGR8。
   */
  bool ConvertToBgr(const cv::Mat& decoded, cv::Mat& bgr)
  {
    if (decoded.empty())
    {
      return false;
    }
    if (decoded.type() == CV_8UC3)
    {
      bgr = decoded;
      return true;
    }
    if (decoded.type() == CV_8UC4)
    {
      cv::cvtColor(decoded, bgr, cv::COLOR_BGRA2BGR);
      return true;
    }
    if (decoded.type() == CV_8UC1)
    {
      cv::cvtColor(decoded, bgr, cv::COLOR_GRAY2BGR);
      return true;
    }

    XR_LOG_ERROR("CaptureFileCamera unsupported cv type=%d", decoded.type());
    return false;
  }

  /**
   * @brief 发布一组原始 IMU 数据。
   */
  void PublishRawImu(const ImuSample& sample)
  {
    GyroStamped gyro_msg{
        .sensor_timestamp_us = sample.timestamp_us,
        .angular_velocity_xyz = sample.gyro_xyz,
    };
    AcclStamped accl_msg{
        .sensor_timestamp_us = sample.timestamp_us,
        .linear_acceleration_xyz = sample.accl_xyz,
    };
    QuatStamped quat_msg{
        .sensor_timestamp_us = sample.timestamp_us,
        .rotation_wxyz = sample.quat_wxyz,
    };

    raw_gyro_topic_.Publish(gyro_msg);
    raw_accl_topic_.Publish(accl_msg);
    raw_quat_topic_.Publish(quat_msg);
    imu_published_.fetch_add(1);
  }

  /**
   * @brief 把 BGR 图像写入当前可写槽位并提交。
   */
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

  /**
   * @brief 处理一帧图像和对应 IMU 数据。
   */
  bool ProcessFrame(const cv::Mat& decoded, const ImuSample& imu, uint64_t period_us)
  {
    cv::Mat bgr;
    if (!ConvertToBgr(decoded, bgr))
    {
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
      const double scaled_ms =
          static_cast<double>(period_us) /
          (static_cast<double>(microseconds_per_millisecond) *
           runtime_.replay_speed);
      const uint64_t sleep_ms = std::max<uint64_t>(
          1U, static_cast<uint64_t>(std::ceil(std::min<double>(
                  scaled_ms, static_cast<double>(UINT32_MAX)))));
      LibXR::Thread::Sleep(static_cast<uint32_t>(sleep_ms));
    }

    return committed;
  }

  /**
   * @brief 回放线程主循环。
   */
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

  /**
   * @brief `LibXR::Thread` 入口。
   */
  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  std::string file_path_{};  ///< 视频文件路径。
  std::string imu_csv_path_{};  ///< IMU CSV 路径。

  RuntimeParam runtime_{};  ///< 运行参数。
  std::vector<ImuSample> imu_samples_{};  ///< 按帧序号索引的 IMU 数据。

  LibXR::RuntimeStringView<> gyro_topic_name_{};  ///< `<camera_name>_gyro`。
  LibXR::RuntimeStringView<> accl_topic_name_{};  ///< `<camera_name>_accl`。
  LibXR::RuntimeStringView<> quat_topic_name_{};  ///< `<camera_name>_quat`。

  LibXR::Topic raw_gyro_topic_{};  ///< gyro topic。
  LibXR::Topic raw_accl_topic_{};  ///< accl topic。
  LibXR::Topic raw_quat_topic_{};  ///< quat topic。

  LibXR::Thread capture_thread_{};  ///< 回放线程。

  std::atomic<bool> running_{false};  ///< 采集线程退出标志。

  std::atomic<uint32_t> frames_committed_{0};  ///< 已提交图像帧数。

  std::atomic<uint32_t> imu_published_{0};  ///< 已发布原始 IMU 组数。

  uint64_t fallback_period_us_{default_period_us};  ///< 缺少相邻时间戳差值时的限速回退值。
};
