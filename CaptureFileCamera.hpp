#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 回放相机：按录制时间回放统一录像，直接发布图像与同步帧 / Replay camera that plays a unified recording at its recorded pace and publishes the images and the synced frames directly
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: QDU-Robomaster/AutoAimTypes
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "AutoAimTypes.hpp"
#include "CameraBase.hpp"
#include "CaptureFileRecording.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"

/// 回放设置，与 YAML 一一对应 / Replay settings, one-to-one with the YAML.
struct ReplaySettings
{
  std::string_view recording_dir;
  double speed;  ///< 1.0 为录制速度，0 为不限速 / 1.0 = recorded pace, 0 = no pacing
  bool loop;     ///< 播完从头开始 / Restart at the end
  uint32_t max_frames;  ///< 0 为不限 / 0 = unlimited
};

/**
 * @brief 回放相机。构造时读完并检查整个录像；每帧发布图像 `<name>_image`，再用同一张图
 *        和录制的 IMU 发布同步帧 `<name>_synced`（录像没有 IMU 时用静止姿态）。几何取自
 *        录像。回放配置里没有 CameraFrameSync。
 *        Replay camera. The whole recording is read and checked at construction. Each
 *        frame publishes the image `<name>_image` and then the synced frame
 *        `<name>_synced` from the same image and the recorded IMU (a resting attitude
 *        when the recording has none). The geometry comes from the recording. Replay
 *        configurations have no CameraFrameSync.
 *
 * 没有空图像槽时等待，回放不丢帧。循环播放时时间戳逐轮平移，保持递增。
 * With no free slot it waits, so replay drops no frames. When looping, timestamps are
 * shifted on every pass so they keep increasing.
 */
class CaptureFileCamera : public CameraBase
{
 public:
  CaptureFileCamera(const CameraTypes::CameraCalibration& calibration,
                    std::string_view name, const ReplaySettings& settings)
      : CameraBase(calibration, {0.5, 0.5}, name, SlotPolicy::WAIT),
        dir_(settings.recording_dir),
        speed_(settings.speed),
        loop_(settings.loop),
        max_frames_(settings.max_frames),
        synced_topic_(LibXR::Topic::CreateTopic<const AutoAim::SyncedFrame*>(
            StageTopicName(name, AutoAim::STAGE_SYNCED).c_str()))
  {
    REQUIRE(speed_ >= 0.0);
    REQUIRE(Load());
    StartCapture();
  }

  ~CaptureFileCamera() override { StopCapture(); }

 private:
  /// 录像没有 IMU 时的静止姿态（时间戳逐帧填写）/ Resting attitude for recordings
  /// without IMU (the timestamp is filled per frame).
  static constexpr AutoAim::ImuSample RESTING_IMU{
      {}, {1.0F, 0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 9.80665F}};
  /// 等待时检查是否停止的间隔 / How often waits check for a stop request.
  static constexpr auto WAIT_STEP = std::chrono::milliseconds(50);

  bool Load()
  {
    std::string error;
    if (!CaptureFileRecording::LoadFrames(dir_, rows_, error))
    {
      XR_LOG_ERROR("%s: %s", Name().c_str(), error.c_str());
      return false;
    }
    if (rows_.empty())
    {
      XR_LOG_ERROR("%s: %s has no frames", Name().c_str(), dir_.c_str());
      return false;
    }
    for (std::size_t i = 0; i < rows_.size(); ++i)
    {
      const CaptureFileRecording::Row& row = rows_[i];
      if (!CameraTypes::GeometryInsideSensor(row.geometry, Calibration()) ||
          (i > 0 && row.timestamp_us < rows_[i - 1].timestamp_us) ||
          !std::filesystem::exists(CaptureFileRecording::PgmPath(dir_, row.frame)))
      {
        XR_LOG_ERROR("%s: frame %u has a bad geometry, a decreasing timestamp or no PGM",
                     Name().c_str(), static_cast<unsigned>(row.frame));
        return false;
      }
    }
    // 一轮的时长：首尾间隔加一个平均帧间隔 / One pass: first-to-last plus one period.
    const uint64_t first_to_last = rows_.back().timestamp_us - rows_.front().timestamp_us;
    pass_us_ =
        first_to_last + (rows_.size() > 1 ? first_to_last / (rows_.size() - 1) : 0);
    XR_LOG_INFO("%s: replaying %u frames from %s (%s IMU)", Name().c_str(),
                static_cast<unsigned>(rows_.size()), dir_.c_str(),
                rows_.front().imu ? "recorded" : "resting");
    return true;
  }

  bool GrabFrame(ImageFrame& frame) override
  {
    if (next_ == rows_.size() && loop_)
    {
      next_ = 0;
      offset_us_ += pass_us_;
    }
    if (next_ == rows_.size() || (max_frames_ != 0 && played_ == max_frames_))
    {
      return Finish();
    }
    const CaptureFileRecording::Row& row = rows_[next_++];
    const uint64_t timestamp_us = row.timestamp_us + offset_us_;
    if (!WaitUntil(timestamp_us))
    {
      return false;
    }

    std::string error;
    if (!CaptureFileRecording::ReadPgm(CaptureFileRecording::PgmPath(dir_, row.frame),
                                       frame.data, error))
    {
      XR_LOG_ERROR("%s: %s", Name().c_str(), error.c_str());
      return false;
    }
    frame.timestamp_us = LibXR::MicrosecondTimestamp(timestamp_us);
    frame.frame_counter = row.frame_counter;
    frame.geometry = row.geometry;
    imu_ = row.imu.value_or(RESTING_IMU);
    imu_.timestamp_us = frame.timestamp_us;
    ++played_;
    return true;
  }

  /// 图像发布后，用同一张图发布同步帧 / After the image, publish the synced frame
  /// holding the same image.
  void OnPublished(const SharedFrame& frame) override
  {
    const AutoAim::SyncedFrame synced{played_, frame, imu_};
    const AutoAim::SyncedFrame* payload = &synced;
    synced_topic_.Publish(payload);
  }

  /// 回放的几何由录像决定；视角请求被接受但不生效。
  /// Replay geometry follows the recording; view requests are accepted and ignored.
  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry&) override
  {
    return LibXR::ErrorCode::OK;
  }

  /// 按 speed 等到该帧的回放时刻；停止请求时返回 false / Wait until the frame is due.
  bool WaitUntil(uint64_t timestamp_us)
  {
    if (speed_ == 0.0)
    {
      return CaptureRunning();
    }
    const auto now = std::chrono::steady_clock::now();
    if (played_ == 0)
    {
      start_wall_ = now;
      start_us_ = timestamp_us;
    }
    const auto due = start_wall_ + std::chrono::microseconds(static_cast<int64_t>(
                                       (timestamp_us - start_us_) / speed_));
    while (CaptureRunning() && std::chrono::steady_clock::now() < due)
    {
      std::this_thread::sleep_until(
          std::min(due, std::chrono::steady_clock::now() + WAIT_STEP));
    }
    return CaptureRunning();
  }

  /// 播完：打印一次，然后停在这里直到采集停止 / Done: log once, then wait for the stop.
  bool Finish()
  {
    if (!finished_)
    {
      finished_ = true;
      XR_LOG_PASS("%s: replay finished after %u frames", Name().c_str(),
                  static_cast<unsigned>(played_));
    }
    while (CaptureRunning())
    {
      std::this_thread::sleep_for(WAIT_STEP);
    }
    return false;
  }

  const std::string dir_;
  const double speed_;
  const bool loop_;
  const uint32_t max_frames_;
  LibXR::Topic synced_topic_;
  std::vector<CaptureFileRecording::Row> rows_;
  AutoAim::ImuSample imu_{};  ///< 当前帧的 IMU，取帧与发布都在采集线程 / Capture thread
  std::size_t next_ = 0;
  uint64_t played_ = 0;
  uint64_t pass_us_ = 0;
  uint64_t offset_us_ = 0;
  uint64_t start_us_ = 0;
  std::chrono::steady_clock::time_point start_wall_{};
  bool finished_ = false;
};
