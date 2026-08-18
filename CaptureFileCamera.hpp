#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 文件回放相机，发布统一 raw frame-bin 内录包与原始 IMU 数据
constructor_args:
  - calibration:
      native_width: 1440
      native_height: 1080
      camera_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [-0.091821039187099038, 0.46399073468302049, 0.0026098786426372819, 0.0009819586010405485, -0.47512788503104569]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 0.0, 1.0, 0.0]
  - runtime:
      file_path: "capture_frames.bin"
      frame_csv_path: "capture_frames.csv"
      imu_csv_path: "capture_imu.csv"
      camera_name: "camera"
      image_topic_name: "camera_image"
      imu_topic_name: "camera_imu"
      realtime: true
      loop: false
      max_frames: 0
      trigger_period_us: 10000
      geometry:
        width: 720
        height: 540
        step: 2160
        roi_offset_x_native: 0
        roi_offset_y_native: 0
        decimation_x: 2
        decimation_y: 2
        flags: CameraTypes::FRAME_GEOMETRY_NONE
        reserved: 0
        sample_phase_x_native: 0.0
        sample_phase_y_native: 0.0
template_args:
  - Layout:
      width: 720
      height: 540
      step: 2160
      encoding: CameraTypes::Encoding::BGR8
required_hardware: []
depends:
  - qdu-future/CameraBase
  - xrobot-org/DurationStatistics
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <opencv2/core.hpp>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "AutoAimReplayBenchmark.hpp"
#include "CameraBase.hpp"
#include "CaptureFileCameraFrameBin.hpp"
#include "CaptureFileCameraInput.hpp"
#include "CaptureFileCameraVideo.hpp"
#include "DurationStatistics.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"

/**
 * @class CaptureFileCamera
 * @brief 文件相机源，用统一 raw frame-bin 内录包按 CameraBase 接口发布数据。
 *
 * 当前仓库主口径优先使用 `frames.bin + frames.csv + imu.csv`；当 `frame_csv_path`
 * 为空时，仍允许受控退回到历史 `video + imu.csv` 回放面，主要用于保留旧 replay
 * 产物的验证能力。
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CaptureFileCamera : public LibXR::Application, public CameraBase<FrameLayoutV>
{
 public:
  using Self = CaptureFileCamera<FrameLayoutV>;  ///< 当前模板实例类型。
  using Base = CameraBase<FrameLayoutV>;         ///< 图像发布基类。
  using ImageFrame = typename Base::ImageFrame;  ///< CameraBase 图像帧类型。
  using CameraCalibration = typename Base::CameraCalibration;  ///< 原生相机标定。
  using FrameGeometry = typename Base::FrameGeometry;          ///< 固定回放采样几何。
  using ProfileId = typename Base::ProfileId;                  ///< 固定档位标识。
  using CameraProfile = typename Base::CameraProfile;          ///< 固定档位描述。
  using AppliedProfile = typename Base::AppliedProfile;        ///< 已应用档位快照。
  using ImuVector = Eigen::Matrix<float, 3, 1>;  ///< 原始 gyro/accl topic 的三轴数据。
  using ImuSample = CaptureFileCameraDetail::ImuSample;      ///< CSV 中的一帧 IMU 数据。
  using FrameRecord = CaptureFileCameraDetail::FrameRecord;  ///< 帧索引 CSV 中的一行。
  using FrameBinReplayFrame =
      CaptureFileCameraDetail::FrameBinReplayFrame;  ///< 已完成图像和 IMU 对齐的回放项。
  using QuatSample = LibXR::Quaternion<float>;       ///< 原始 quat topic 的四元数数据。

  /**
   * @brief 编译期帧存储布局，来自 BSP YAML 预设。
   */
  static inline constexpr auto frame_layout = Base::frame_layout;

  /**
   * @brief BGR8 图像通道数。
   */
  static constexpr int channel_count = 3;

  /**
   * @brief 单行图像字节数。
   */
  static constexpr std::size_t frame_step = static_cast<std::size_t>(frame_layout.step);

  /**
   * @brief 图像宽度，单位像素。
   */
  static constexpr int frame_width = static_cast<int>(frame_layout.width);

  /**
   * @brief 图像高度，单位像素。
   */
  static constexpr int frame_height = static_cast<int>(frame_layout.height);

  static_assert(frame_layout.encoding == CameraTypes::Encoding::BGR8,
                "CaptureFileCamera currently publishes BGR8 frames");
  static_assert(frame_step ==
                    static_cast<std::size_t>(frame_layout.width) * channel_count,
                "CaptureFileCamera expects tightly packed BGR8 frames");

  /**
   * @struct RuntimeParam
   * @brief xrobot YAML 传入的运行时参数。
   */
  struct RuntimeParam
  {
    std::string_view file_path = "capture_frames.bin";  ///< 帧数据 bin 路径。
    std::string_view frame_csv_path =
        "capture_frames.csv";  ///< 统一 raw frame-bin 包的帧索引 CSV。
    std::string_view imu_csv_path =
        "capture_imu.csv";  ///< 与 frames.csv 同步对齐的 IMU CSV。
    std::string_view camera_name =
        "camera";  ///< CameraBase 相机名，也是原始 IMU 话题前缀。
    std::string_view image_topic_name =
        "camera_image";  ///< 图像话题，供 CameraFrameSync 消费。
    std::string_view imu_topic_name = "camera_imu";  ///< 同步后 IMU 话题名。
    bool realtime = true;                            ///< 是否按录制帧间隔限速播放。
    bool loop = false;                               ///< EOF 后是否回到第 0 帧继续播放。
    uint32_t max_frames = 0;             ///< 0 表示不限帧数，测试可用环境变量覆盖。
    uint32_t trigger_period_us = 10000;  ///< 单档回放对应的图像触发周期。
    FrameGeometry geometry{
        .width = frame_layout.width,
        .height = frame_layout.height,
        .step = frame_layout.step,
        .roi_offset_x_native = 0,
        .roi_offset_y_native = 0,
        .decimation_x = 2,
        .decimation_y = 2,
        .flags = CameraTypes::FRAME_GEOMETRY_NONE,
        .reserved = 0,
        .sample_phase_x_native = 0.0F,
        .sample_phase_y_native = 0.0F,
    };  ///< 整次回放固定复制到每帧的原生采样几何。
  };

  /**
   * @brief 构造文件相机，检查输入包后启动后台回放线程。
   *
   * @param hw 硬件容器，透传给 CameraBase。
   * @param app 应用管理器，用于注册监控回调。
   * @param calibration 原生传感器坐标系下的不可变相机标定。
   * @param runtime 文件路径、话题名和回放控制参数。
   */
  explicit CaptureFileCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                             CameraCalibration calibration, RuntimeParam runtime)
      : Base(hw, calibration, runtime.camera_name, runtime.image_topic_name,
             runtime.imu_topic_name),
        file_path_(runtime.file_path),
        frame_csv_path_(runtime.frame_csv_path),
        imu_csv_path_(runtime.imu_csv_path),
        runtime_(runtime),
        frame_geometry_(runtime.geometry),
        gyro_topic_name_(this->NameView(), "_gyro"),
        accl_topic_name_(this->NameView(), "_accl"),
        quat_topic_name_(this->NameView(), "_quat"),
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<ImuVector>(gyro_topic_name_.CStr())),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<ImuVector>(accl_topic_name_.CStr())),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatSample>(quat_topic_name_.CStr()))
  {
    if (!CameraTypes::ValidateFrameGeometry(frame_layout, this->Calibration(),
                                            frame_geometry_))
    {
      XR_LOG_ERROR("CaptureFileCamera invalid fixed FrameGeometry");
      throw std::runtime_error("CaptureFileCamera: invalid frame geometry");
    }
    if (runtime_.trigger_period_us == 0U)
    {
      throw std::runtime_error("CaptureFileCamera: trigger period must be non-zero");
    }
    profiles_[0] = {.id = ProfileId::WIDE,
                    .geometry = frame_geometry_,
                    .trigger_period_us = runtime_.trigger_period_us};
    ApplyEnvironmentOverrides();
    LoadImuCsv();
    if (IsFrameBinMode())
    {
      LoadFrameCsv();
      BuildFrameBinReplayPlan();
      ValidateFrameBin();
    }
    else
    {
      ValidateLegacyVideo();
    }
    running_.store(true);
    capture_thread_ = std::thread(CaptureThreadMain, this);
    app.Register(*this);
  }

  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept override
  {
    return profiles_;
  }

  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    if (id != profiles_[0].id)
    {
      return LibXR::ErrorCode::NOT_SUPPORT;
    }
    applied = {.id = profiles_[0].id, .geometry = profiles_[0].geometry};
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 通知采集线程在下一轮循环退出。
   */
  ~CaptureFileCamera() override
  {
    running_.store(false);
    if (capture_thread_.joinable())
    {
      capture_thread_.join();
    }
  }

  /**
   * @brief 周期性输出回放状态。
   */
  void OnMonitor() override
  {
    const auto video_read = video_read_duration_.GetSummary();
    const auto bgr_convert = bgr_convert_duration_.GetSummary();
    const auto imu_publish = imu_publish_duration_.GetSummary();
    const auto image_commit = image_commit_duration_.GetSummary();
    const auto replay_sleep = replay_sleep_duration_.GetSummary();
    const uint32_t period_frames = period_frames_committed_.exchange(0);
    XR_LOG_INFO("CaptureFileCamera monitor: frames=%u imu=%u running=%d period_frames=%u",
                frames_committed_.load(), imu_published_.load(), running_.load() ? 1 : 0,
                period_frames);
    XR_LOG_INFO(
        "CaptureFileCamera video_read count=%llu average_us=%llu minimum_us=%llu "
        "maximum_us=%llu",
        static_cast<unsigned long long>(video_read.sample_count),
        static_cast<unsigned long long>(video_read.average_us),
        static_cast<unsigned long long>(video_read.minimum_us),
        static_cast<unsigned long long>(video_read.maximum_us));
    XR_LOG_INFO(
        "CaptureFileCamera bgr_convert count=%llu average_us=%llu minimum_us=%llu "
        "maximum_us=%llu",
        static_cast<unsigned long long>(bgr_convert.sample_count),
        static_cast<unsigned long long>(bgr_convert.average_us),
        static_cast<unsigned long long>(bgr_convert.minimum_us),
        static_cast<unsigned long long>(bgr_convert.maximum_us));
    XR_LOG_INFO(
        "CaptureFileCamera imu_publish count=%llu average_us=%llu minimum_us=%llu "
        "maximum_us=%llu",
        static_cast<unsigned long long>(imu_publish.sample_count),
        static_cast<unsigned long long>(imu_publish.average_us),
        static_cast<unsigned long long>(imu_publish.minimum_us),
        static_cast<unsigned long long>(imu_publish.maximum_us));
    XR_LOG_INFO(
        "CaptureFileCamera image_commit count=%llu average_us=%llu minimum_us=%llu "
        "maximum_us=%llu",
        static_cast<unsigned long long>(image_commit.sample_count),
        static_cast<unsigned long long>(image_commit.average_us),
        static_cast<unsigned long long>(image_commit.minimum_us),
        static_cast<unsigned long long>(image_commit.maximum_us));
    XR_LOG_INFO(
        "CaptureFileCamera replay_sleep count=%llu average_us=%llu minimum_us=%llu "
        "maximum_us=%llu",
        static_cast<unsigned long long>(replay_sleep.sample_count),
        static_cast<unsigned long long>(replay_sleep.average_us),
        static_cast<unsigned long long>(replay_sleep.minimum_us),
        static_cast<unsigned long long>(replay_sleep.maximum_us));
  }

  /**
   * @brief 文件相机不支持曝光控制，此接口用于满足 CameraBase 合约。
   */
  void SetExposure(double) override {}

  /**
   * @brief 文件相机不支持增益控制，此接口用于满足 CameraBase 合约。
   */
  void SetGain(double) override {}

 private:
  /**
   * @brief 根据相邻 IMU timestamp 推导当前帧回放间隔。
   */
  uint64_t ReplayPeriodUsForIndex(std::size_t index) const
  {
    const auto& samples = IsFrameBinMode() ? frame_bin_imu_samples_ : imu_samples_;
    if (index > 0 && index < samples.size() &&
        samples[index].timestamp_us > samples[index - 1].timestamp_us)
    {
      return samples[index].timestamp_us - samples[index - 1].timestamp_us;
    }
    return CaptureFileCameraDetail::default_period_us;
  }

  /**
   * @brief 当前是否按帧数据 bin 模式回放。
   */
  bool IsFrameBinMode() const { return !frame_csv_path_.empty(); }

  void ValidateLegacyVideo()
  {
    if (!CaptureFileCameraDetail::ReadVideoInfo(file_path_, video_info_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open legacy video '%s'",
                   file_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open legacy video");
    }
    if (video_info_.width != frame_width || video_info_.height != frame_height)
    {
      XR_LOG_ERROR(
          "CaptureFileCamera legacy video shape mismatch: got=%dx%d expected=%dx%d",
          video_info_.width, video_info_.height, frame_width, frame_height);
      throw std::runtime_error("CaptureFileCamera: legacy video shape mismatch");
    }
    XR_LOG_PASS("CaptureFileCamera opened legacy video=%s width=%d height=%d fps=%.3f",
                file_path_.c_str(), video_info_.width, video_info_.height,
                video_info_.fps);
  }

  /**
   * @brief 按录制间隔限速；非实时模式直接返回。
   */
  void SleepReplayPeriodUntil(uint64_t target_timestamp_us)
  {
    if (!runtime_.realtime)
    {
      return;
    }

    const uint64_t now_us = static_cast<uint64_t>(LibXR::Thread::GetTime()) *
                            CaptureFileCameraDetail::microseconds_per_millisecond;
    if (target_timestamp_us <= now_us)
    {
      return;
    }

    const uint64_t remaining_us = target_timestamp_us - now_us;
    const uint64_t sleep_ms =
        remaining_us / CaptureFileCameraDetail::microseconds_per_millisecond;
    if (sleep_ms == 0U)
    {
      return;
    }
    auto replay_sleep_measurement = replay_sleep_duration_.Measure();
    LibXR::Thread::Sleep(static_cast<uint32_t>(sleep_ms));
  }

  /**
   * @brief 校验解码后的图像是否满足编译期帧布局约束。
   */
  static bool FrameShapeMatches(const cv::Mat& bgr)
  {
    return bgr.cols == frame_width && bgr.rows == frame_height &&
           bgr.elemSize() == channel_count;
  }

  /**
   * @brief 应用测试环境变量覆盖。
   *
   * 环境变量只用于 CI / smoke test 限帧和加速，正常运行配置仍以 YAML 为准。
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
    if (const char* env = std::getenv("CAPTURE_FILE_CAMERA_PLAYBACK_RATE_MILLI"))
    {
      uint32_t parsed = 0U;
      if (!CaptureFileCameraDetail::ParsePlaybackRateMilli(env, parsed))
      {
        XR_LOG_ERROR("CaptureFileCamera invalid playback rate milli: '%s'", env);
        throw std::runtime_error("CaptureFileCamera: invalid playback rate milli");
      }
      playback_rate_milli_ = parsed;
    }
    XR_LOG_INFO("CaptureFileCamera playback rate milli=%u", playback_rate_milli_);
  }

  /**
   * @brief 加载显式 IMU CSV，运行时按帧索引取样。
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
        if (input.eof())
        {
          XR_LOG_WARN("CaptureFileCamera ignored truncated IMU csv tail line %u",
                      line_number);
          break;
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
   * @brief 加载帧索引 CSV。
   */
  void LoadFrameCsv()
  {
    std::ifstream input(frame_csv_path_);
    if (!input.is_open())
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open frame csv '%s'",
                   frame_csv_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open frame csv");
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

      FrameRecord frame{};
      if (!CaptureFileCameraDetail::ParseFrameCsvRow(line, frame))
      {
        if (frame_records_.empty() && !header_skipped)
        {
          header_skipped = true;
          continue;
        }
        if (input.eof())
        {
          XR_LOG_WARN("CaptureFileCamera ignored truncated frame csv tail line %u",
                      line_number);
          break;
        }
        XR_LOG_ERROR("CaptureFileCamera invalid frame csv line %u", line_number);
        throw std::runtime_error("CaptureFileCamera: invalid frame csv line");
      }
      frame_records_.push_back(frame);
    }

    if (frame_records_.empty())
    {
      XR_LOG_ERROR("CaptureFileCamera frame csv is empty: '%s'", frame_csv_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: empty frame csv");
    }
  }

  /**
   * @brief 统一内录包只允许未压缩 `BGR8 raw` 图像帧。
   */
  void ValidateFrameRecordsAreRawBgr() const
  {
    for (const auto& frame : frame_records_)
    {
      if (!CaptureFileCameraDetail::FrameRecordIsRawBgr(frame, Base::image_bytes))
      {
        XR_LOG_ERROR(
            "CaptureFileCamera only accepts raw BGR8 frame records: frame=%u codec=%s "
            "size=%u expected=%u",
            static_cast<unsigned>(frame.frame_index), frame.codec.c_str(),
            static_cast<unsigned>(frame.size_bytes),
            static_cast<unsigned>(Base::image_bytes));
        throw std::runtime_error("CaptureFileCamera: non-raw frame record");
      }
    }
  }

  /**
   * @brief 将帧索引和 IMU CSV 按 timestamp 对齐成回放计划。
   */
  void BuildFrameBinReplayPlan()
  {
    std::unordered_map<uint64_t, ImuSample> imu_by_timestamp;
    imu_by_timestamp.reserve(imu_samples_.size());
    for (const auto& imu : imu_samples_)
    {
      imu_by_timestamp[imu.timestamp_us] = imu;
    }

    frame_bin_replay_frames_.clear();
    frame_bin_imu_samples_.clear();
    frame_bin_replay_frames_.reserve(frame_records_.size());
    frame_bin_imu_samples_.reserve(frame_records_.size());

    for (const auto& frame : frame_records_)
    {
      auto imu_it = imu_by_timestamp.find(frame.timestamp_us);
      if (imu_it == imu_by_timestamp.end())
      {
        continue;
      }
      frame_bin_replay_frames_.push_back(
          FrameBinReplayFrame{.frame = frame, .imu = imu_it->second});
      frame_bin_imu_samples_.push_back(imu_it->second);
    }

    if (frame_bin_replay_frames_.empty())
    {
      XR_LOG_ERROR("CaptureFileCamera frame bin has no timestamp-aligned frames");
      throw std::runtime_error("CaptureFileCamera: frame bin has no aligned frames");
    }
  }

  /**
   * @brief 校验 frames bin 是否包含回放计划要求的完整图像。
   */
  void ValidateFrameBin()
  {
    uint64_t file_size = 0;
    if (!CaptureFileCameraDetail::ValidateFrameBinFile(
            file_path_, frame_bin_replay_frames_, file_size))
    {
      XR_LOG_ERROR("CaptureFileCamera invalid frames bin '%s'", file_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: invalid frames bin");
    }

    const auto skipped =
        static_cast<unsigned>(frame_records_.size() - frame_bin_replay_frames_.size());
    XR_LOG_PASS(
        "CaptureFileCamera opened frame bin=%s bytes=%u frames=%u aligned=%u skipped=%u",
        file_path_.c_str(), static_cast<unsigned>(file_size),
        static_cast<unsigned>(frame_records_.size()),
        static_cast<unsigned>(frame_bin_replay_frames_.size()), skipped);
  }

  /**
   * @brief 等待回放产品显式完成整条处理链构造。
   */
  void WaitForPipelineReady()
  {
    if (running_.load() && !AutoAimReplayBenchmark::WaitForPipelineReady())
    {
      XR_LOG_ERROR("CaptureFileCamera timed out waiting for replay pipeline readiness");
      running_.store(false);
    }
  }

  /**
   * @brief 发布一组原始 gyro/accl/quat topic。
   *
   * 采样时间戳写入 Topic 元信息，消息内容只保留测量值。
   */
  void PublishRawImu(const ImuSample& sample)
  {
    auto imu_publish_measurement = imu_publish_duration_.Measure();
    ImuVector gyro_msg;
    ImuVector accl_msg;
    gyro_msg << sample.gyro_xyz[0], sample.gyro_xyz[1], sample.gyro_xyz[2];
    accl_msg << sample.accl_xyz[0], sample.accl_xyz[1], sample.accl_xyz[2];
    QuatSample quat_msg(sample.quat_wxyz[0], sample.quat_wxyz[1], sample.quat_wxyz[2],
                        sample.quat_wxyz[3]);
    const LibXR::MicrosecondTimestamp timestamp(sample.timestamp_us);

    raw_gyro_topic_.Publish(gyro_msg, timestamp);
    raw_accl_topic_.Publish(accl_msg, timestamp);
    raw_quat_topic_.Publish(quat_msg, timestamp);
    imu_published_.fetch_add(1);
  }

  /**
   * @brief 把 BGR 图像写入 CameraBase 图像缓冲区并提交。
   */
  bool WriteAndCommitImage(const cv::Mat& bgr, uint64_t timestamp_us)
  {
    auto image_commit_measurement = image_commit_duration_.Measure();
    const auto commit_begin = std::chrono::steady_clock::now();
    ImageFrame* image = CaptureFileCameraDetail::WaitForReplaySlot(
        [this]() { return this->GetWritableImage(); },
        [this]() { return running_.load(); }, []() { LibXR::Thread::Sleep(1); });
    if (image == nullptr)
    {
      return false;
    }

    image->timestamp_us = timestamp_us;
    image->geometry = frame_geometry_;
    if (bgr.isContinuous())
    {
      std::memcpy(image->data.data(), bgr.data, Base::image_bytes);
    }
    else
    {
      for (int row = 0; row < frame_height; ++row)
      {
        std::memcpy(image->data.data() + static_cast<std::size_t>(row) * frame_step,
                    bgr.ptr(row), frame_step);
      }
    }

    if (!this->CommitImage())
    {
      XR_LOG_WARN("CaptureFileCamera image commit failed");
      return false;
    }

    frames_committed_.fetch_add(1);
    period_frames_committed_.fetch_add(1);
    const auto commit_end = std::chrono::steady_clock::now();
    AutoAimReplayBenchmark::RecordCaptureCommit(
        timestamp_us,
        std::chrono::duration<double, std::milli>(commit_end - commit_begin).count());
    return true;
  }

  /**
   * @brief 从 frames bin 读取一帧图像并解码为 BGR。
   */
  bool ReadFrameBinFrame(CaptureFileCameraDetail::FrameBinInput& frames,
                         const FrameBinReplayFrame& replay, cv::Mat& bgr)
  {
    const auto read_begin = std::chrono::steady_clock::now();
    std::vector<uint8_t> frame_bytes;
    bool read_ok = false;
    {
      auto video_read_measurement = video_read_duration_.Measure();
      read_ok = frames.Read(replay.frame, frame_bytes);
    }
    if (!read_ok)
    {
      XR_LOG_ERROR("CaptureFileCamera frame bytes read failed index=%u",
                   static_cast<unsigned>(replay.frame.frame_index));
      return false;
    }
    const auto read_finish = std::chrono::steady_clock::now();

    bool decode_ok = false;
    {
      auto bgr_convert_measurement = bgr_convert_duration_.Measure();
      decode_ok = CaptureFileCameraDetail::DecodeFrameBytes(
          replay.frame, frame_bytes, Base::image_bytes, frame_width, frame_height,
          frame_step, bgr);
    }
    if (!decode_ok)
    {
      XR_LOG_ERROR("CaptureFileCamera frame decode failed index=%u codec=%s size=%u",
                   static_cast<unsigned>(replay.frame.frame_index),
                   replay.frame.codec.c_str(), static_cast<unsigned>(frame_bytes.size()));
      return false;
    }
    const auto decode_finish = std::chrono::steady_clock::now();
    if (!FrameShapeMatches(bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera encoded frame shape mismatch index=%u",
                   static_cast<unsigned>(replay.frame.frame_index));
      return false;
    }
    AutoAimReplayBenchmark::RecordCaptureDecode(
        replay.frame.timestamp_us,
        std::chrono::duration<double, std::milli>(read_finish - read_begin).count(),
        std::chrono::duration<double, std::milli>(decode_finish - read_finish).count());
    return true;
  }

  /**
   * @brief 帧数据 bin 回放路径。
   *
   * 按帧索引从 frames bin 中读取图像，解码后写入 CameraBase。
   */
  void RunFrameBinReplay()
  {
    WaitForPipelineReady();
    CaptureFileCameraDetail::FrameBinInput frames;
    if (!frames.Open(file_path_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open frames bin '%s' in worker",
                   file_path_.c_str());
      running_.store(false);
      AutoAimReplayBenchmark::MarkSourceComplete(frames_committed_.load(), false);
      return;
    }

    std::size_t frame_index = 0;
    bool replay_ok = false;
    uint64_t wall_start_us = 0;
    const uint64_t replay_start_us =
        frame_bin_replay_frames_.empty()
            ? 0U
            : frame_bin_replay_frames_.front().imu.timestamp_us;
    while (running_.load())
    {
      if (frame_index >= frame_bin_replay_frames_.size())
      {
        XR_LOG_PASS("CaptureFileCamera reached frame bin EOF after %u committed frames",
                    frames_committed_.load());
        if (!runtime_.loop)
        {
          replay_ok = true;
          running_.store(false);
          break;
        }
        frame_index = 0;
        continue;
      }

      const FrameBinReplayFrame& replay = frame_bin_replay_frames_[frame_index];
      cv::Mat bgr;
      if (!ReadFrameBinFrame(frames, replay, bgr))
      {
        running_.store(false);
        break;
      }

      if (runtime_.realtime && replay_start_us != 0U)
      {
        if (wall_start_us == 0U)
        {
          wall_start_us = static_cast<uint64_t>(LibXR::Thread::GetTime()) *
                          CaptureFileCameraDetail::microseconds_per_millisecond;
        }
        uint64_t target_wall_us = 0U;
        if (!CaptureFileCameraDetail::TryReplayDeadlineUs(
                wall_start_us, replay.imu.timestamp_us - replay_start_us,
                playback_rate_milli_, target_wall_us))
        {
          XR_LOG_ERROR("CaptureFileCamera replay deadline overflow rate_milli=%u",
                       playback_rate_milli_);
          running_.store(false);
          break;
        }
        SleepReplayPeriodUntil(target_wall_us);
      }

      AutoAimReplayBenchmark::RecordCaptureStart(replay.frame.timestamp_us);
      PublishRawImu(replay.imu);
      if (!WriteAndCommitImage(bgr, replay.frame.timestamp_us))
      {
        running_.store(false);
        break;
      }
      if (!AutoAimReplayBenchmark::WaitForAimer(replay.frame.timestamp_us))
      {
        XR_LOG_ERROR("CaptureFileCamera timed out waiting for aimer timestamp=%llu",
                     static_cast<unsigned long long>(replay.frame.timestamp_us));
        running_.store(false);
        break;
      }

      ++frame_index;
      if (runtime_.max_frames != 0U && frames_committed_.load() >= runtime_.max_frames)
      {
        XR_LOG_PASS("CaptureFileCamera reached max_frames=%u", runtime_.max_frames);
        replay_ok = true;
        running_.store(false);
        break;
      }
    }
    AutoAimReplayBenchmark::MarkSourceComplete(frames_committed_.load(), replay_ok);
  }

  void RunLegacyVideoReplay()
  {
    WaitForPipelineReady();
    CaptureFileCameraDetail::VideoInput video;
    if (!video.Open(file_path_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open legacy video '%s' in worker",
                   file_path_.c_str());
      running_.store(false);
      return;
    }

    std::size_t frame_index = 0;
    const uint64_t wall_start_us = static_cast<uint64_t>(LibXR::Thread::GetTime()) *
                                   CaptureFileCameraDetail::microseconds_per_millisecond;
    const uint64_t replay_start_us =
        imu_samples_.empty() ? 0U : imu_samples_.front().timestamp_us;
    while (running_.load())
    {
      if (frame_index >= imu_samples_.size())
      {
        XR_LOG_PASS(
            "CaptureFileCamera reached legacy video EOF after %u committed frames",
            frames_committed_.load());
        if (!runtime_.loop)
        {
          running_.store(false);
          break;
        }
        video.Rewind();
        frame_index = 0;
        continue;
      }

      cv::Mat decoded;
      {
        auto video_read_measurement = video_read_duration_.Measure();
        if (!video.Read(decoded))
        {
          XR_LOG_PASS(
              "CaptureFileCamera video reader reached EOF after %u committed frames",
              frames_committed_.load());
          if (!runtime_.loop)
          {
            running_.store(false);
            break;
          }
          video.Rewind();
          continue;
        }
      }

      cv::Mat bgr;
      {
        auto bgr_convert_measurement = bgr_convert_duration_.Measure();
        if (!CaptureFileCameraDetail::ConvertToBgr(decoded, bgr) ||
            !FrameShapeMatches(bgr))
        {
          XR_LOG_ERROR(
              "CaptureFileCamera legacy video frame shape/type mismatch index=%u",
              static_cast<unsigned>(frame_index));
          running_.store(false);
          break;
        }
      }

      const auto& imu = imu_samples_[frame_index];
      PublishRawImu(imu);
      if (!WriteAndCommitImage(bgr, imu.timestamp_us))
      {
        running_.store(false);
        break;
      }

      if (runtime_.realtime && replay_start_us != 0U)
      {
        const uint64_t target_wall_us =
            wall_start_us + (imu.timestamp_us - replay_start_us);
        SleepReplayPeriodUntil(target_wall_us);
      }

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
   * @brief 采集线程入口的实际回放分发逻辑。
   */
  void RunReplay()
  {
    if (IsFrameBinMode())
    {
      RunFrameBinReplay();
    }
    else
    {
      RunLegacyVideoReplay();
    }
  }

  /**
   * @brief LibXR 线程入口适配函数。
   */
  static void CaptureThreadMain(Self* self) { self->RunReplay(); }

 private:
  std::string file_path_{};       ///< RuntimeParam 是 string_view，这里持有路径副本。
  std::string frame_csv_path_{};  ///< 帧索引 CSV 路径。
  std::string imu_csv_path_{};    ///< 显式 IMU CSV 路径副本。

  RuntimeParam runtime_{};  ///< 应用环境变量覆盖后的运行时参数。
  uint32_t playback_rate_milli_ =
      CaptureFileCameraDetail::default_playback_rate_milli;  ///< wall-clock 倍率。
  FrameGeometry frame_geometry_{};            ///< 构造时验证并逐帧复制的固定采样几何。
  std::array<CameraProfile, 1U> profiles_{};  ///< 生命周期内稳定的单档描述。
  CaptureFileCameraDetail::VideoInfo video_info_{};  ///< legacy video 模式下的视频信息。
  std::vector<ImuSample> imu_samples_{};             ///< CSV 中加载的全部 IMU 数据。
  std::vector<FrameRecord> frame_records_{};         ///< 帧索引 CSV 内容。
  std::vector<FrameBinReplayFrame> frame_bin_replay_frames_{};  ///< bin 模式下已对齐帧。
  std::vector<ImuSample> frame_bin_imu_samples_{};  ///< bin 模式下用于限速的 IMU 序列。

  LibXR::RuntimeStringView<> gyro_topic_name_{};  ///< `<camera_name>_gyro`。
  LibXR::RuntimeStringView<> accl_topic_name_{};  ///< `<camera_name>_accl`。
  LibXR::RuntimeStringView<> quat_topic_name_{};  ///< `<camera_name>_quat`。

  LibXR::Topic raw_gyro_topic_{};  ///< CameraFrameSync 消费的原始陀螺话题。
  LibXR::Topic raw_accl_topic_{};  ///< CameraFrameSync 消费的原始加速度话题。
  LibXR::Topic raw_quat_topic_{};  ///< CameraFrameSync 消费的原始姿态话题。

  std::thread capture_thread_{};  ///< 解码和发布线程。

  std::atomic<bool> running_{false};  ///< 采集线程退出标志。

  std::atomic<uint32_t> frames_committed_{0};         ///< 已提交图像帧数。
  std::atomic<uint32_t> period_frames_committed_{0};  ///< 本监控周期已提交图像帧数。

  std::atomic<uint32_t> imu_published_{0};  ///< 已发布原始 IMU 组数。
  XRobot::DurationStatistics video_read_duration_{};
  XRobot::DurationStatistics bgr_convert_duration_{};
  XRobot::DurationStatistics imu_publish_duration_{};
  XRobot::DurationStatistics image_commit_duration_{};
  XRobot::DurationStatistics replay_sleep_duration_{};
};
