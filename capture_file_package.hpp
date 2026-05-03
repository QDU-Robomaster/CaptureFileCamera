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
static constexpr uint64_t default_period_us = 10000;
static constexpr double microseconds_per_second = 1000000.0;
static constexpr uint64_t microseconds_per_millisecond = 1000;

struct ImuSample
{
  uint64_t timestamp_us{};
  std::array<float, 4> quat_wxyz{};
  std::array<float, 3> gyro_xyz{};
  std::array<float, 3> accl_xyz{};
};

struct VideoInfo
{
  int width{};
  int height{};
  double fps{};
  uint64_t fallback_period_us{default_period_us};
};

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
