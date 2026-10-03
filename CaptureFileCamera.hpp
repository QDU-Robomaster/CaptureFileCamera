#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 文件回放相机：回放内录的图像与 IMU 数据并发布 / File replay camera that publishes recorded images and IMU data
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: xrobot-org/DurationStatistics
  ref: same-or-dev
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
#include <limits>
#include <opencv2/core.hpp>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "CameraBase.hpp"
#include "CaptureFileCameraFrameBin.hpp"
#include "CaptureFileCameraInput.hpp"
#include "CaptureFileCameraVideo.hpp"
#include "DurationStatistics.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/**
 * @class CaptureFileCamera
 * @brief 文件回放相机：从内录包读取图像与 IMU 数据，按 CameraBase 接口发布。
 *        File replay camera that reads images and IMU data from a recording and
 *        publishes them through the CameraBase interface.
 *
 * 内录包为 `frames.bin + frames.csv + imu.csv`；`frame_csv_path` 为空时，按
 * `video + imu.csv` 回放。构造时加载并校验输入，随后启动后台回放线程；回放线程对每一帧
 * 先发布一组原始 IMU，再提交对应的图像。
 * A recording is `frames.bin + frames.csv + imu.csv`; with an empty `frame_csv_path` it
 * is replayed as `video + imu.csv`. The construction loads and validates the input and
 * then starts a background replay thread, which for every frame publishes one raw IMU
 * group and then commits the matching image.
 *
 * @tparam FrameLayoutV 帧布局，紧密排列的 BGR8。
 *                      Frame layout, tightly packed BGR8.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CaptureFileCamera : public CameraBase<FrameLayoutV>
{
 public:
  /// 当前模板实例类型。
  /// The current template instance type.
  using Self = CaptureFileCamera<FrameLayoutV>;
  /// 图像发布基类。
  /// Image publishing base class.
  using Base = CameraBase<FrameLayoutV>;
  /// CameraBase 图像帧类型。
  /// CameraBase image frame type.
  using ImageFrame = typename Base::ImageFrame;
  /// 原生相机标定。
  /// Native camera calibration.
  using CameraCalibration = typename Base::CameraCalibration;
  /// 固定回放采样几何。
  /// Fixed replay sampling geometry.
  using FrameGeometry = typename Base::FrameGeometry;
  /// 固定档位标识。
  /// Fixed profile identifier.
  using ProfileId = typename Base::ProfileId;
  /// 固定档位描述。
  /// Fixed profile description.
  using CameraProfile = typename Base::CameraProfile;
  /// 已应用档位快照。
  /// Applied profile snapshot.
  using AppliedProfile = typename Base::AppliedProfile;
  /// 原始 gyro/accl Topic 的三轴数据。
  /// Three-axis data of the raw gyro/accl Topics.
  using ImuVector = Eigen::Matrix<float, 3, 1>;
  /// CSV 中的一行 IMU 数据。
  /// One IMU row of the CSV.
  using ImuSample = CaptureFileCameraDetail::ImuSample;
  /// 帧索引 CSV 中的一行。
  /// One row of the frame index CSV.
  using FrameRecord = CaptureFileCameraDetail::FrameRecord;
  /// 已完成图像和 IMU 对齐的回放项。
  /// Replay item with the image and IMU aligned.
  using FrameBinReplayFrame = CaptureFileCameraDetail::FrameBinReplayFrame;
  /// 原始 quat Topic 的四元数数据。
  /// Quaternion data of the raw quat Topic.
  using QuatSample = LibXR::Quaternion<float>;

  /// 编译期帧存储布局，取自模板参数 `FrameLayoutV`。
  /// Compile-time frame storage layout, taken from the template parameter
  /// `FrameLayoutV`.
  static inline constexpr auto frame_layout = Base::frame_layout;

  /// BGR8 图像通道数。
  /// Number of channels of a BGR8 image.
  static constexpr int channel_count = 3;

  /// 单行图像字节数。
  /// Bytes per image row.
  static constexpr std::size_t frame_step = static_cast<std::size_t>(frame_layout.step);

  /// 图像宽度，单位像素。
  /// Image width in pixels.
  static constexpr int frame_width = static_cast<int>(frame_layout.width);

  /// 图像高度，单位像素。
  /// Image height in pixels.
  static constexpr int frame_height = static_cast<int>(frame_layout.height);

  /// 省略触发周期时采用的默认周期，单位微秒。
  /// Default trigger period in microseconds, used when none is given.
  static constexpr uint32_t default_trigger_period_us = 10000U;

  static_assert(frame_layout.encoding == CameraTypes::Encoding::BGR8,
                "CaptureFileCamera currently publishes BGR8 frames");
  static_assert(frame_step ==
                    static_cast<std::size_t>(frame_layout.width) * channel_count,
                "CaptureFileCamera expects tightly packed BGR8 frames");

  /**
   * @struct RuntimeParam
   * @brief 运行时参数：文件路径、Topic 名称和回放控制。
   *        Runtime parameters: file paths, Topic names and replay control.
   */
  struct RuntimeParam
  {
    std::string_view file_path = "capture_frames.bin";  ///< 帧数据 bin 或视频路径
    ///< Frame data bin path or video path
    std::string_view frame_csv_path = "capture_frames.csv";  ///< 帧索引 CSV，空为视频
    ///< Frame index CSV path; empty selects video mode
    std::string_view imu_csv_path = "capture_imu.csv";  ///< IMU CSV 路径
    ///< Path of the IMU CSV
    std::string_view camera_name = "camera";  ///< 相机名，兼作前缀与文件名
    ///< Camera name, also the Topic prefix and the command file name
    std::string_view image_topic_name = "camera_image";  ///< 图像 Topic 名称
    ///< Name of the image Topic
    std::string_view imu_topic_name = "camera_imu";  ///< 传给 CameraBase 的 IMU Topic
    ///< Synchronized IMU Topic name passed to CameraBase
    bool realtime = true;  ///< 按录制时间戳限速回放
    ///< Rate limit the replay by the recorded timestamps
    bool loop = false;  ///< 到达文件末尾后从头开始
    ///< Restart from the beginning at the end of the file
    uint32_t max_frames = 0;  ///< 最大提交帧数，0 表示不限制
    ///< Maximum number of committed frames; 0 means unlimited
    uint32_t trigger_period_us = default_trigger_period_us;  ///< 触发周期 (us)
    ///< Image trigger period of the single profile (us), non-zero
    FrameGeometry geometry{
        frame_layout.width,
        frame_layout.height,
        frame_layout.step,
        0U,
        0U,
        2U,
        2U,
        CameraTypes::FRAME_GEOMETRY_NONE,
        0U,
        0.0F,
        0.0F,
    };  ///< 整次回放固定复制到每帧的原生采样几何
    ///< Native sampling geometry copied to every frame for the whole replay
    double replay_speed = 1.0;  ///< 回放倍率，有限正数，只改变播放节奏
    ///< Replay speed factor, a finite positive number; only the replay pace changes

    /**
     * @brief 默认构造，所有字段取默认值。
     *        Default construction with all fields at their defaults.
     */
    RuntimeParam() = default;

    /**
     * @brief 按字段顺序给出全部参数，含触发周期。
     *        Construct from all fields in order, including the trigger period.
     *
     * @param file_path_in 帧数据 bin 路径或视频路径。
     *                     Frame data bin path or video path.
     * @param frame_csv_path_in 帧索引 CSV 路径。
     *                          Frame index CSV path.
     * @param imu_csv_path_in IMU CSV 路径。
     *                        IMU CSV path.
     * @param camera_name_in 相机名。
     *                       Camera name.
     * @param image_topic_name_in 图像 Topic 名称。
     *                            Image Topic name.
     * @param imu_topic_name_in 同步 IMU Topic 名称。
     *                          Synchronized IMU Topic name.
     * @param realtime_in 是否限速回放。
     *                    Whether to rate limit the replay.
     * @param loop_in 是否循环播放。
     *                Whether to loop.
     * @param max_frames_in 最大提交帧数，0 表示不限制。
     *                      Maximum number of committed frames; 0 means unlimited.
     * @param trigger_period_us_in 图像触发周期，单位微秒。
     *                             Image trigger period in microseconds.
     * @param geometry_in 固定采样几何。
     *                    Fixed sampling geometry.
     * @param replay_speed_in 回放倍率。
     *                        Replay speed factor.
     */
    constexpr RuntimeParam(std::string_view file_path_in,
                           std::string_view frame_csv_path_in,
                           std::string_view imu_csv_path_in,
                           std::string_view camera_name_in,
                           std::string_view image_topic_name_in,
                           std::string_view imu_topic_name_in, bool realtime_in,
                           bool loop_in, uint32_t max_frames_in,
                           uint32_t trigger_period_us_in, FrameGeometry geometry_in,
                           double replay_speed_in = 1.0)
        : file_path(file_path_in),
          frame_csv_path(frame_csv_path_in),
          imu_csv_path(imu_csv_path_in),
          camera_name(camera_name_in),
          image_topic_name(image_topic_name_in),
          imu_topic_name(imu_topic_name_in),
          realtime(realtime_in),
          loop(loop_in),
          max_frames(max_frames_in),
          trigger_period_us(trigger_period_us_in),
          geometry(geometry_in),
          replay_speed(replay_speed_in)
    {
    }

    /**
     * @brief 省略触发周期，`geometry_in` 紧跟 `max_frames_in`，触发周期取
     *        `default_trigger_period_us`。
     *        Omit the trigger period: `geometry_in` directly follows `max_frames_in`
     *        and the trigger period is `default_trigger_period_us`.
     *
     * @param file_path_in 帧数据 bin 路径或视频路径。
     *                     Frame data bin path or video path.
     * @param frame_csv_path_in 帧索引 CSV 路径。
     *                          Frame index CSV path.
     * @param imu_csv_path_in IMU CSV 路径。
     *                        IMU CSV path.
     * @param camera_name_in 相机名。
     *                       Camera name.
     * @param image_topic_name_in 图像 Topic 名称。
     *                            Image Topic name.
     * @param imu_topic_name_in 同步 IMU Topic 名称。
     *                          Synchronized IMU Topic name.
     * @param realtime_in 是否限速回放。
     *                    Whether to rate limit the replay.
     * @param loop_in 是否循环播放。
     *                Whether to loop.
     * @param max_frames_in 最大提交帧数，0 表示不限制。
     *                      Maximum number of committed frames; 0 means unlimited.
     * @param geometry_in 固定采样几何。
     *                    Fixed sampling geometry.
     * @param replay_speed_in 回放倍率。
     *                        Replay speed factor.
     */
    constexpr RuntimeParam(std::string_view file_path_in,
                           std::string_view frame_csv_path_in,
                           std::string_view imu_csv_path_in,
                           std::string_view camera_name_in,
                           std::string_view image_topic_name_in,
                           std::string_view imu_topic_name_in, bool realtime_in,
                           bool loop_in, uint32_t max_frames_in,
                           FrameGeometry geometry_in, double replay_speed_in = 1.0)
        : RuntimeParam(file_path_in, frame_csv_path_in, imu_csv_path_in, camera_name_in,
                       image_topic_name_in, imu_topic_name_in, realtime_in, loop_in,
                       max_frames_in, default_trigger_period_us, geometry_in,
                       replay_speed_in)
    {
    }
  };

  /**
   * @brief 返回默认的原生相机标定：1440x1080，PLUMB_BOB 畸变模型。
   *        Return the default native camera calibration: 1440x1080, PLUMB_BOB
   *        distortion model.
   *
   * @return 默认标定。
   *         The default calibration.
   */
  static CameraCalibration DefaultCalibration() { return {.native_width = 1440, .native_height = 1080, .camera_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {-0.09182103918709904, 0.4639907346830205, 0.002609878642637282, 0.0009819586010405485, -0.4751278850310457}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 0.0, 1.0, 0.0}}; }

  /**
   * @brief 返回默认运行时参数。
   *        Return the default runtime parameters.
   *
   * @return 全部字段取默认值的 `RuntimeParam`。
   *         A `RuntimeParam` with all fields at their defaults.
   */
  static RuntimeParam DefaultRuntime() { return {}; }

  /**
   * @brief 构造文件相机，加载并校验内录包后启动后台回放线程。
   *        Construct the file camera, load and validate the recording, and start the
   *        background replay thread.
   *
   * 几何、触发周期、回放倍率、CSV 与图像数据任一校验失败时抛出
   * `std::runtime_error`。
   * A `std::runtime_error` is thrown when the geometry, trigger period,
   * replay speed, CSVs or image data fail validation.
   *
   * @param ramfs CameraBase 注册相机命令文件的 RamFS。
   *              RamFS in which CameraBase registers the camera command file.
   * @param calibration 原生传感器坐标系下的相机标定。
   *                    Camera calibration in the native sensor coordinate system.
   * @param runtime 文件路径、Topic 名称和回放控制参数。
   *                File paths, Topic names and replay control parameters.
   */
  explicit CaptureFileCamera(
      LibXR::RamFS& ramfs,
      CameraCalibration calibration = DefaultCalibration(),
      RuntimeParam runtime = DefaultRuntime())
      : Base(ramfs, calibration, runtime.camera_name, runtime.image_topic_name,
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
    if (!CaptureFileCameraDetail::ValidReplaySpeed(runtime_.replay_speed))
    {
      XR_LOG_ERROR("CaptureFileCamera invalid replay_speed=%.6f", runtime_.replay_speed);
      throw std::runtime_error(
          "CaptureFileCamera: replay_speed must be finite and positive");
    }
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
  }

  /**
   * @brief 返回固定档位列表，仅含 `WIDE` 一档。
   *        Return the fixed profile list, which holds the single profile `WIDE`.
   *
   * @return 档位描述。
   *         Profile descriptions.
   */
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept override
  {
    return profiles_;
  }

  /**
   * @brief 应用档位：`WIDE` 返回其 geometry，其他档位返回 `NOT_SUPPORT`。
   *        Apply a profile: `WIDE` returns its per-frame geometry and any other profile
   *        returns `NOT_SUPPORT`.
   *
   * @param id 档位标识。
   *           Profile identifier.
   * @param applied 生效的档位快照。
   *                Applied profile snapshot.
   * @return `OK` 或 `NOT_SUPPORT`。
   *         `OK` or `NOT_SUPPORT`.
   */
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
   * @brief 通知回放线程在下一轮循环退出并等待其结束。
   *        Ask the replay thread to exit at its next iteration and wait for it.
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
   * @brief 输出回放状态：累计提交帧数、已发布 IMU 组数、运行状态、本周期提交帧数，
   *        以及各阶段耗时统计（单位微秒）。
   *        Print the replay status: total committed frames, published IMU groups, running
   *        state, frames committed in the period, and per-stage timing statistics
   *        (in microseconds).
   */
  void OnMonitor()
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
   * @brief 空实现，满足 CameraBase 接口；传入的曝光值被忽略。
   *        Empty implementation that satisfies the CameraBase interface; the given
   *        exposure value is ignored.
   */
  void SetExposure(double) override {}

  /**
   * @brief 空实现，满足 CameraBase 接口；传入的增益值被忽略。
   *        Empty implementation that satisfies the CameraBase interface; the given
   *        gain value is ignored.
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
  bool SleepReplayPeriodUntil(uint64_t target_timestamp_us)
  {
    if (!runtime_.realtime)
    {
      return running_.load();
    }

    const uint64_t now_us = static_cast<uint64_t>(LibXR::Thread::GetTime()) *
                            CaptureFileCameraDetail::microseconds_per_millisecond;
    if (target_timestamp_us <= now_us)
    {
      return running_.load();
    }

    const uint64_t remaining_us = target_timestamp_us - now_us;
    const uint64_t sleep_ms =
        remaining_us / CaptureFileCameraDetail::microseconds_per_millisecond +
        (remaining_us % CaptureFileCameraDetail::microseconds_per_millisecond != 0U);
    if (sleep_ms > std::numeric_limits<uint32_t>::max())
    {
      XR_LOG_ERROR("CaptureFileCamera replay wait exceeds millisecond timer range");
      running_.store(false);
      return false;
    }
    auto replay_sleep_measurement = replay_sleep_duration_.Measure();
    LibXR::Thread::Sleep(static_cast<uint32_t>(sleep_ms));
    return running_.load();
  }

  /** @brief 每轮首帧重建播放起点，在对应帧发布前等待。 */
  bool PaceReplayFrame(std::size_t frame_index, uint64_t timestamp_us,
                       uint64_t replay_start_us, uint64_t& wall_start_us)
  {
    if (!runtime_.realtime)
    {
      return running_.load();
    }
    if (frame_index == 0U)
    {
      wall_start_us = static_cast<uint64_t>(LibXR::Thread::GetTime()) *
                      CaptureFileCameraDetail::microseconds_per_millisecond;
    }
    uint64_t target_wall_us = 0U;
    if (timestamp_us < replay_start_us ||
        !CaptureFileCameraDetail::TryReplayDeadlineUs(
            wall_start_us, timestamp_us - replay_start_us, runtime_.replay_speed,
            target_wall_us))
    {
      XR_LOG_ERROR("CaptureFileCamera invalid replay deadline speed=%.6f",
                   runtime_.replay_speed);
      running_.store(false);
      return false;
    }
    return SleepReplayPeriodUntil(target_wall_us);
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
   * 环境变量只用于 CI / smoke test 限帧和关闭限速；倍率由 YAML 配置。
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
   * @brief 检查帧索引记录均为未压缩 `BGR8 raw` 图像帧，否则抛出异常。
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
    return true;
  }

  /**
   * @brief 从 frames bin 读取一帧图像并解码为 BGR。
   */
  bool ReadFrameBinFrame(CaptureFileCameraDetail::FrameBinInput& frames,
                         const FrameBinReplayFrame& replay, cv::Mat& bgr)
  {
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
    if (!FrameShapeMatches(bgr))
    {
      XR_LOG_ERROR("CaptureFileCamera encoded frame shape mismatch index=%u",
                   static_cast<unsigned>(replay.frame.frame_index));
      return false;
    }
    return true;
  }

  /**
   * @brief 帧数据 bin 回放路径。
   *
   * 按帧索引从 frames bin 中读取图像，解码后写入 CameraBase。
   */
  void RunFrameBinReplay()
  {
    CaptureFileCameraDetail::FrameBinInput frames;
    if (!frames.Open(file_path_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open frames bin '%s' in worker",
                   file_path_.c_str());
      running_.store(false);
      return;
    }

    std::size_t frame_index = 0;
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

      if (!PaceReplayFrame(frame_index, replay.imu.timestamp_us, replay_start_us,
                           wall_start_us))
      {
        break;
      }

      PublishRawImu(replay.imu);
      if (!WriteAndCommitImage(bgr, replay.frame.timestamp_us))
      {
        running_.store(false);
        break;
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

  void RunLegacyVideoReplay()
  {
    CaptureFileCameraDetail::VideoInput video;
    if (!video.Open(file_path_))
    {
      XR_LOG_ERROR("CaptureFileCamera failed to open legacy video '%s' in worker",
                   file_path_.c_str());
      running_.store(false);
      return;
    }

    std::size_t frame_index = 0;
    uint64_t wall_start_us = 0U;
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
          frame_index = 0;
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
      if (!PaceReplayFrame(frame_index, imu.timestamp_us, replay_start_us, wall_start_us))
      {
        break;
      }
      PublishRawImu(imu);
      if (!WriteAndCommitImage(bgr, imu.timestamp_us))
      {
        running_.store(false);
        break;
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

  RuntimeParam runtime_{};                    ///< 应用环境变量覆盖后的运行时参数。
  FrameGeometry frame_geometry_{};            ///< 构造时验证并逐帧复制的固定采样几何。
  std::array<CameraProfile, 1U> profiles_{};  ///< 生命周期内稳定的单档描述。
  CaptureFileCameraDetail::VideoInfo video_info_{};  ///< 视频模式下的视频信息。
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
