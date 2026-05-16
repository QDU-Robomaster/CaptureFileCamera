#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "CaptureFileCameraInput.hpp"

namespace CaptureFileCameraDetail
{
/// 读取视频文件的基本信息。
inline bool ReadVideoInfo(const std::string& path, VideoInfo& info)
{
  cv::VideoCapture cap(path);
  if (!cap.isOpened())
  {
    return false;
  }

  info.width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  info.height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  info.fps = cap.get(cv::CAP_PROP_FPS);
  if (info.fps > 0.0 && std::isfinite(info.fps))
  {
    info.fallback_period_us =
        static_cast<uint64_t>(std::llround(microseconds_per_second / info.fps));
  }
  return true;
}

/// 普通视频文件读取器。
class VideoInput
{
 public:
  bool Open(const std::string& path) { return capture_.open(path); }

  bool Read(cv::Mat& decoded) { return capture_.read(decoded); }

  void Rewind() { capture_.set(cv::CAP_PROP_POS_FRAMES, 0.0); }

 private:
  cv::VideoCapture capture_{};
};
}  // namespace CaptureFileCameraDetail
