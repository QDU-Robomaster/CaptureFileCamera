#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 文件回放相机，发布视频帧与原始 IMU 数据
constructor_args:
  - runtime:
      file_path: "capture.avi"
      imu_csv_path: "capture_imu.csv"
      camera_name: "camera"
      image_topic_name: "camera_image"
      imu_topic_name: "camera_imu"
      realtime: true
      loop: false
      max_frames: 0
      frame_csv_path: ""
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
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "CaptureFileCameraFrameBin.hpp"
#include "CaptureFileCameraInput.hpp"
#include "CaptureFileCameraVideo.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"

/**
 * @class CaptureFileCamera
 * @brief 文件相机源，用普通视频或帧数据 bin 按 CameraBase 接口发布数据。
 *
 * 视频文件只包含图像，IMU CSV 显式给出每帧的 timestamp/quat/gyro/accl。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CaptureFileCamera : public LibXR::Application,
                          public CameraBase<CameraInfoV>
{
 public:
  using Self = CaptureFileCamera<CameraInfoV>;  ///< 当前模板实例类型。
  using Base = CameraBase<CameraInfoV>;  ///< 图像发布基类。
  using ImageFrame = typename Base::ImageFrame;  ///< CameraBase 图像帧类型。
  using ImuVector = Eigen::Matrix<float, 3, 1>;  ///< 原始 gyro/accl topic 的三轴数据。
  using ImuSample = CaptureFileCameraDetail::ImuSample;  ///< CSV 中的一帧 IMU 数据。
  using FrameRecord = CaptureFileCameraDetail::FrameRecord;  ///< 帧索引 CSV 中的一行。
  using FrameBinReplayFrame =
      CaptureFileCameraDetail::FrameBinReplayFrame;  ///< 已完成图像和 IMU 对齐的回放项。
  using QuatSample = LibXR::Quaternion<float>;  ///< 原始 quat topic 的四元数数据。

  /**
   * @brief 编译期相机模型，来自 BSP YAML 预设。
   */
  static inline constexpr auto camera_info = Base::camera_info;

  /**
   * @brief BGR8 图像通道数。
   */
  static constexpr int channel_count = 3;

  /**
   * @brief 单行图像字节数。
   */
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);

  /**
   * @brief 图像宽度，单位像素。
   */
  static constexpr int frame_width = static_cast<int>(camera_info.width);

  /**
   * @brief 图像高度，单位像素。
   */
  static constexpr int frame_height = static_cast<int>(camera_info.height);

  /**
   * @brief 等待 CameraFrameSync 接入图像 sink 时的日志周期。
   */
  static constexpr uint32_t image_sink_wait_log_ms = 1000;

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
    std::string_view file_path = "capture.avi";  ///< 视频路径或帧数据 bin 路径。
    std::string_view imu_csv_path = "capture_imu.csv";  ///< 同帧 IMU CSV。
    std::string_view camera_name = "camera";  ///< CameraBase 相机名，也是原始 IMU 话题前缀。
    std::string_view image_topic_name = "camera_image";  ///< 图像话题，供 CameraFrameSync 消费。
    std::string_view imu_topic_name = "camera_imu";  ///< 同步后 IMU 话题名。
    bool realtime = true;  ///< 是否按录制帧间隔限速播放。
    bool loop = false;  ///< EOF 后是否回到第 0 帧继续播放。
    uint32_t max_frames = 0;  ///< 0 表示不限帧数，测试可用环境变量覆盖。
    std::string_view frame_csv_path = "";  ///< 非空时按帧数据 bin 回放。
  };

  /**
   * @brief 构造文件相机，检查输入包后启动后台回放线程。
   *
   * @param hw 硬件容器，透传给 CameraBase。
   * @param app 应用管理器，用于注册监控回调。
   * @param runtime 文件路径、话题名和回放控制参数。
   */
  explicit CaptureFileCamera(LibXR::HardwareContainer& hw,
                             LibXR::ApplicationManager& app,
                             RuntimeParam runtime)
      : Base(hw, runtime.camera_name, runtime.image_topic_name, runtime.imu_topic_name),
        file_path_(runtime.file_path),
        frame_csv_path_(runtime.frame_csv_path),
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
    LoadImuCsv();
    if (IsFrameBinMode())
    {
      LoadFrameCsv();
      BuildFrameBinReplayPlan();
      ValidateFrameBin();
    }
    else
    {
      ValidateVideo();
    }
    running_.store(true);
    capture_thread_ = std::thread(CaptureThreadMain, this);
    app.Register(*this);
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
    XR_LOG_INFO("CaptureFileCamera monitor: frames=%u imu=%u running=%d",
                frames_committed_.load(), imu_published_.load(), running_.load() ? 1 : 0);
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
    return video_info_.fallback_period_us;
  }

  /**
   * @brief 当前是否按帧数据 bin 模式回放。
   */
  bool IsFrameBinMode() const { return !frame_csv_path_.empty(); }

  /**
   * @brief 按录制间隔限速；非实时模式直接返回。
   */
  void SleepReplayPeriod(uint64_t period_us) const
  {
    if (!runtime_.realtime)
    {
      return;
    }

    const uint64_t sleep_ms = std::max<uint64_t>(
        1U, period_us / CaptureFileCameraDetail::microseconds_per_millisecond);
    LibXR::Thread::Sleep(static_cast<uint32_t>(sleep_ms));
  }

  /**
   * @brief 校验解码后的图像是否满足 CameraInfo 编译期约束。
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
  }

  /**
   * @brief 构造期预打开视频文件，提前发现路径和分辨率错误。
   */
  void ValidateVideo()
  {
    if (!CaptureFileCameraDetail::ReadVideoInfo(file_path_, video_info_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open '%s'", file_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: failed to open file");
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
      XR_LOG_ERROR("CaptureFileCamera frame csv is empty: '%s'",
                   frame_csv_path_.c_str());
      throw std::runtime_error("CaptureFileCamera: empty frame csv");
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
    XR_LOG_PASS("CaptureFileCamera opened frame bin=%s bytes=%u frames=%u aligned=%u skipped=%u",
                file_path_.c_str(), static_cast<unsigned>(file_size),
                static_cast<unsigned>(frame_records_.size()),
                static_cast<unsigned>(frame_bin_replay_frames_.size()), skipped);
  }

  /**
   * @brief 等 CameraFrameSync 接好图像 sink，避免启动阶段白丢前几帧。
   */
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

  /**
   * @brief 发布一组原始 gyro/accl/quat topic。
   *
   * 采样时间戳写入 Topic 元信息，消息内容只保留测量值。
   */
  void PublishRawImu(const ImuSample& sample)
  {
    ImuVector gyro_msg;
    ImuVector accl_msg;
    gyro_msg << sample.gyro_xyz[0], sample.gyro_xyz[1], sample.gyro_xyz[2];
    accl_msg << sample.accl_xyz[0], sample.accl_xyz[1], sample.accl_xyz[2];
    QuatSample quat_msg(sample.quat_wxyz[0], sample.quat_wxyz[1],
                        sample.quat_wxyz[2], sample.quat_wxyz[3]);
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
   * @brief 从 frames bin 读取一帧图像，解码后提交到 CameraBase。
   */
  bool ReadAndCommitFrameBinFrame(CaptureFileCameraDetail::FrameBinInput& frames,
                                  const FrameBinReplayFrame& replay)
  {
    std::vector<uint8_t> frame_bytes;
    if (!frames.Read(replay.frame, frame_bytes))
    {
      XR_LOG_ERROR("CaptureFileCamera frame bytes read failed index=%u",
                   static_cast<unsigned>(replay.frame.frame_index));
      return false;
    }

    cv::Mat bgr;
    if (!CaptureFileCameraDetail::DecodeFrameBytes(replay.frame, frame_bytes,
                                                   Base::image_bytes, frame_width,
                                                   frame_height, frame_step, bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera frame decode failed index=%u codec=%s size=%u",
                   static_cast<unsigned>(replay.frame.frame_index),
                   replay.frame.codec.c_str(),
                   static_cast<unsigned>(frame_bytes.size()));
      return false;
    }
    if (!FrameShapeMatches(bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera encoded frame shape mismatch index=%u",
                   static_cast<unsigned>(replay.frame.frame_index));
      return false;
    }
    return WriteAndCommitImage(bgr, replay.frame.timestamp_us);
  }

  /**
   * @brief 处理视频模式的一帧图像。
   *
   * 按同一帧索引发布 IMU 和图像；限速只复现回放节奏。
   */
  bool ProcessVideoFrame(const cv::Mat& decoded, const ImuSample& imu,
                         uint64_t period_us)
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

    SleepReplayPeriod(period_us);

    return committed;
  }

  /**
   * @brief 视频回放路径：OpenCV 解码图像，IMU CSV 按帧索引对齐。
   */
  void RunVideoReplay()
  {
    WaitForImageSink();
    CaptureFileCameraDetail::VideoInput video;
    if (!video.Open(file_path_))
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
        video.Rewind();
        frame_index = 0;
        continue;
      }

      if (!video.Read(decoded))
      {
        XR_LOG_PASS("CaptureFileCamera reached EOF after %u committed frames",
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

      const uint64_t period_us = ReplayPeriodUsForIndex(frame_index);
      (void)ProcessVideoFrame(decoded, imu_samples_[frame_index], period_us);
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
   * @brief 帧数据 bin 回放路径。
   *
   * 按帧索引从 frames bin 中读取图像，解码后写入 CameraBase。
   */
  void RunFrameBinReplay()
  {
    WaitForImageSink();
    CaptureFileCameraDetail::FrameBinInput frames;
    if (!frames.Open(file_path_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open frames bin '%s' in worker",
                   file_path_.c_str());
      running_.store(false);
      return;
    }

    std::size_t frame_index = 0;
    while (running_.load())
    {
      if (frame_index >= frame_bin_replay_frames_.size())
      {
        XR_LOG_PASS("CaptureFileCamera reached frame bin EOF after %u committed frames",
                    frames_committed_.load());
        if (!runtime_.loop)
        {
          running_.store(false);
          break;
        }
        frame_index = 0;
        continue;
      }

      const FrameBinReplayFrame& replay = frame_bin_replay_frames_[frame_index];
      PublishRawImu(replay.imu);
      const bool committed = ReadAndCommitFrameBinFrame(frames, replay);
      if (!committed)
      {
        running_.store(false);
        break;
      }

      SleepReplayPeriod(ReplayPeriodUsForIndex(frame_index));

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
      RunVideoReplay();
    }
  }

  /**
   * @brief LibXR 线程入口适配函数。
   */
  static void CaptureThreadMain(Self* self) { self->RunReplay(); }

 private:
  std::string file_path_{};  ///< RuntimeParam 是 string_view，这里持有路径副本。
  std::string frame_csv_path_{};  ///< 帧索引 CSV 路径。
  std::string imu_csv_path_{};  ///< 显式 IMU CSV 路径副本。

  RuntimeParam runtime_{};  ///< 应用环境变量覆盖后的运行时参数。
  std::vector<ImuSample> imu_samples_{};  ///< CSV 中加载的全部 IMU 数据。
  std::vector<FrameRecord> frame_records_{};  ///< 帧索引 CSV 内容。
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

  std::atomic<uint32_t> frames_committed_{0};  ///< 已提交图像帧数。

  std::atomic<uint32_t> imu_published_{0};  ///< 已发布原始 IMU 组数。

  CaptureFileCameraDetail::VideoInfo video_info_{};  ///< 视频几何和限速回退周期。
};
