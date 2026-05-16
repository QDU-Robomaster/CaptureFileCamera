#pragma once

#include <cstdint>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "CaptureFileCameraInput.hpp"

namespace CaptureFileCameraDetail
{
/// 校验帧数据 bin 是否包含所有要回放的帧。
inline bool ValidateFrameBinFile(const std::string& path,
                                 const std::vector<FrameBinReplayFrame>& frames,
                                 uint64_t& file_size)
{
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input.is_open())
  {
    return false;
  }

  const auto file_size_pos = input.tellg();
  if (file_size_pos == std::streampos(-1))
  {
    return false;
  }
  file_size = static_cast<uint64_t>(file_size_pos);

  for (const auto& replay : frames)
  {
    if (replay.frame.size_bytes == 0 || replay.frame.offset_bytes > file_size ||
        replay.frame.size_bytes > file_size - replay.frame.offset_bytes)
    {
      return false;
    }
  }
  return true;
}

/// 帧数据 bin 读取器。
class FrameBinInput
{
 public:
  bool Open(const std::string& path)
  {
    frames_.open(path, std::ios::binary);
    return frames_.is_open();
  }

  bool Read(const FrameRecord& frame, std::vector<uint8_t>& frame_bytes)
  {
    frame_bytes.resize(static_cast<std::size_t>(frame.size_bytes));
    frames_.clear();
    frames_.seekg(static_cast<std::streamoff>(frame.offset_bytes), std::ios::beg);
    frames_.read(reinterpret_cast<char*>(frame_bytes.data()),
                 static_cast<std::streamsize>(frame_bytes.size()));
    return frames_.good();
  }

 private:
  std::ifstream frames_{};
};

/// 把帧数据 bin 中的一帧解成 BGR8。
inline bool DecodeFrameBytes(const FrameRecord& frame,
                             const std::vector<uint8_t>& frame_bytes,
                             std::size_t image_bytes, int width, int height,
                             std::size_t step, cv::Mat& bgr)
{
  if (FrameRecordIsRawBgr(frame, image_bytes))
  {
    if (frame_bytes.size() != image_bytes)
    {
      return false;
    }
    bgr = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(frame_bytes.data()),
                  step);
    return true;
  }

  cv::Mat encoded_mat(1, static_cast<int>(frame_bytes.size()), CV_8UC1,
                      const_cast<uint8_t*>(frame_bytes.data()));
  const cv::Mat decoded = cv::imdecode(encoded_mat, cv::IMREAD_UNCHANGED);
  return ConvertToBgr(decoded, bgr);
}
}  // namespace CaptureFileCameraDetail
