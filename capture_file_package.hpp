#pragma once

#include <array>
#include <cctype>
#include <cmath>
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
 * `timestamp_us` 是传感器侧采样时间戳。发布原始 gyro/accl/quat 时，
 * 该值写入 Topic timestamp，payload 只携带测量数组。
 */
struct ImuSample
{
  uint64_t timestamp_us{};  ///< 传感器侧时间戳，单位微秒。
  std::array<float, 4> quat_wxyz{};  ///< 姿态四元数，顺序为 wxyz。
  std::array<float, 3> gyro_xyz{};  ///< 角速度，单位 rad/s。
  std::array<float, 3> accl_xyz{};  ///< 线加速度，单位 m/s^2。
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
}  // namespace CaptureFileCameraDetail
