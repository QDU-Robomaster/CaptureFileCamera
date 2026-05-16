#pragma once

#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace CaptureFileCameraDetail
{
/// 默认 100 Hz 回放周期；仅在 CSV 相邻时间戳或视频 FPS 不可用时兜底。
static constexpr uint64_t default_period_us = 10000;
static constexpr double microseconds_per_second = 1000000.0;
static constexpr uint64_t microseconds_per_millisecond = 1000;

/**
 * @brief CSV 中的一帧传感器记录。
 *
 * `timestamp_us` 是传感器侧采样时间戳。发布原始 gyro/accl/quat 时，该值写入
 * Topic timestamp，消息内容只携带测量数组。
 */
struct ImuSample
{
  uint64_t timestamp_us{};  ///< 传感器侧时间戳，单位微秒。
  std::array<float, 4> quat_wxyz{};  ///< 姿态四元数，顺序为 wxyz。
  std::array<float, 3> gyro_xyz{};  ///< 角速度，单位 rad/s。
  std::array<float, 3> accl_xyz{};  ///< 线加速度，单位 m/s^2。
};

/// 帧数据 bin 的索引记录。
struct FrameRecord
{
  uint64_t frame_index{};  ///< 录制时的连续帧号。
  uint64_t timestamp_us{};  ///< 图像传感器侧时间戳，单位微秒。
  uint64_t offset_bytes{};  ///< 当前图像在 frames.bin 中的起始偏移。
  uint64_t size_bytes{};  ///< 当前图像的字节数。
  std::string codec{};  ///< 可选图像编码；为空时由大小自动判断。
};

/// 帧数据 bin 回放时的一条已对齐记录。
struct FrameBinReplayFrame
{
  FrameRecord frame{};  ///< 图像位置。
  ImuSample imu{};  ///< 与该图像 timestamp 对齐的 IMU。
};

/// 视频几何信息和回放限速兜底周期。
struct VideoInfo
{
  int width{};  ///< 视频宽度，单位像素。
  int height{};  ///< 视频高度，单位像素。
  double fps{};  ///< OpenCV 读取到的帧率，可能为 0 或无效值。
  uint64_t fallback_period_us{default_period_us};  ///< 无法从 CSV 推导周期时使用。
};

/// 解析一个十进制数字 token，拒绝尾部非空白字符和非有限值。
inline bool ParseNumber(const std::string& token, double& value)
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

/// 空行和 `#` 注释行不参与 CSV 数据解析。
inline bool IsSkippableCsvLine(const std::string& line)
{
  for (char ch : line)
  {
    if (std::isspace(static_cast<unsigned char>(ch)) == 0)
    {
      return ch == '#';
    }
  }
  return true;
}

/// 解析固定列顺序：timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az。
inline bool ParseImuCsvRow(const std::string& line, ImuSample& sample)
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

inline std::string TrimAscii(std::string value)
{
  while (!value.empty() &&
         std::isspace(static_cast<unsigned char>(value.front())) != 0)
  {
    value.erase(value.begin());
  }
  while (!value.empty() &&
         std::isspace(static_cast<unsigned char>(value.back())) != 0)
  {
    value.pop_back();
  }
  return value;
}

inline bool StringEqualsIgnoreCase(const std::string& lhs, const char* rhs)
{
  std::size_t rhs_len = 0;
  while (rhs[rhs_len] != '\0')
  {
    ++rhs_len;
  }
  if (lhs.size() != rhs_len)
  {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i)
  {
    char a = lhs[i];
    char b = rhs[i];
    if (a >= 'A' && a <= 'Z')
    {
      a = static_cast<char>(a - 'A' + 'a');
    }
    if (b >= 'A' && b <= 'Z')
    {
      b = static_cast<char>(b - 'A' + 'a');
    }
    if (a != b)
    {
      return false;
    }
  }
  return true;
}

inline bool CodecIsRaw(const std::string& codec)
{
  return StringEqualsIgnoreCase(codec, "raw");
}

/// 解析列顺序：frame_index,camera_timestamp_us,offset_bytes,size_bytes[,codec]。
inline bool ParseFrameCsvRow(const std::string& line, FrameRecord& frame)
{
  std::array<std::string, 5> tokens{};
  std::stringstream stream(line);
  std::string token;
  std::size_t index = 0;
  while (std::getline(stream, token, ','))
  {
    if (index >= tokens.size())
    {
      return false;
    }
    tokens[index] = token;
    ++index;
  }
  if (index != 4 && index != 5)
  {
    return false;
  }

  std::array<double, 4> values{};
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (!ParseNumber(tokens[i], values[i]))
    {
      return false;
    }
  }

  if (values[0] < 0.0 || values[1] < 0.0 ||
      values[2] < 0.0 || values[3] < 0.0)
  {
    return false;
  }

  frame.frame_index = static_cast<uint64_t>(std::llround(values[0]));
  frame.timestamp_us = static_cast<uint64_t>(std::llround(values[1]));
  frame.offset_bytes = static_cast<uint64_t>(std::llround(values[2]));
  frame.size_bytes = static_cast<uint64_t>(std::llround(values[3]));
  frame.codec = index == 5 ? TrimAscii(tokens[4]) : std::string{};
  return true;
}

/// 将 OpenCV 解码出的常见 8 位图像格式统一成 BGR8。
inline bool ConvertToBgr(const cv::Mat& decoded, cv::Mat& bgr)
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
  return false;
}

/// 判断一条帧索引记录是否是未压缩 BGR8。
inline bool FrameRecordIsRawBgr(const FrameRecord& frame, std::size_t image_bytes)
{
  if (CodecIsRaw(frame.codec))
  {
    return true;
  }
  return frame.codec.empty() && frame.size_bytes == image_bytes;
}
}  // namespace CaptureFileCameraDetail
