#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "CameraTypes.hpp"

/**
 * @brief 统一录像格式的读取。一个录像是一个目录：
 *        Reading the unified recording format. A recording is a directory:
 *
 * - `session.txt`：录制时的相机设置，回放不读 / camera settings, not read by replay
 * - `frames.csv`：`frame,timestamp_us,frame_counter,roi_x,roi_y,decimation`，可选再接
 *   `qw,qx,qy,qz,gx,gy,gz,ax,ay,az`（四元数、rad/s、m/s²，机体系 x 右、y 前、z 上）
 * - `<frame:06>.pgm`：640×512 P5 8 位 BayerRG8，(0,0) 为 R
 */
namespace CaptureFileRecording
{
/// 录像里与一帧同步的 IMU / IMU recorded with one frame.
struct RecordedImu
{
  std::array<float, 4> rotation_wxyz;
  std::array<float, 3> angular_velocity_xyz;
  std::array<float, 3> linear_acceleration_xyz;
};

/// frames.csv 的一行 / One row of frames.csv.
struct Row
{
  uint64_t frame;  ///< 文件序号，对应 `<frame:06>.pgm` / File index
  uint64_t timestamp_us;
  uint32_t frame_counter;
  CameraTypes::FrameGeometry geometry;
  std::optional<RecordedImu> imu;
};

inline std::string PgmPath(const std::string& dir, uint64_t frame)
{
  char name[32];
  std::snprintf(name, sizeof(name), "/%06llu.pgm",
                static_cast<unsigned long long>(frame));
  return dir + name;
}

namespace Detail
{
inline std::vector<std::string> Split(const std::string& line)
{
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true)
  {
    const std::size_t comma = line.find(',', begin);
    fields.push_back(line.substr(begin, comma - begin));
    if (comma == std::string::npos)
    {
      return fields;
    }
    begin = comma + 1;
  }
}

inline bool ParseUnsigned(const std::string& text, uint64_t& value)
{
  char* end = nullptr;
  value = std::strtoull(text.c_str(), &end, 10);
  return !text.empty() && *end == '\0';
}

inline bool ParseFloat(const std::string& text, float& value)
{
  char* end = nullptr;
  value = std::strtof(text.c_str(), &end);
  return !text.empty() && *end == '\0';
}
}  // namespace Detail

/**
 * @brief 读 `<dir>/frames.csv`。列按表头名查找；IMU 十列要么都有要么都没有。
 *        Read `<dir>/frames.csv`. Columns are found by header name; the ten IMU
 *        columns are either all present or all absent.
 * @return 成功返回 true；失败时 `error` 说明原因 / false with a reason in `error`.
 */
inline bool LoadFrames(const std::string& dir, std::vector<Row>& rows, std::string& error)
{
  static constexpr std::array<const char*, 6> REQUIRED = {
      "frame", "timestamp_us", "frame_counter", "roi_x", "roi_y", "decimation"};
  static constexpr std::array<const char*, 10> IMU = {"qw", "qx", "qy", "qz", "gx",
                                                      "gy", "gz", "ax", "ay", "az"};
  std::ifstream file(dir + "/frames.csv");
  std::string line;
  if (!file || !std::getline(file, line))
  {
    error = "cannot read " + dir + "/frames.csv";
    return false;
  }
  if (!line.empty() && line.back() == '\r')
  {
    line.pop_back();
  }
  const std::vector<std::string> header = Detail::Split(line);
  const auto column = [&header](const char* name) -> int
  {
    for (std::size_t i = 0; i < header.size(); ++i)
    {
      if (header[i] == name)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  std::array<int, REQUIRED.size()> required{};
  for (std::size_t i = 0; i < REQUIRED.size(); ++i)
  {
    required[i] = column(REQUIRED[i]);
    if (required[i] < 0)
    {
      error = std::string("frames.csv has no column ") + REQUIRED[i];
      return false;
    }
  }
  std::array<int, IMU.size()> imu{};
  std::size_t imu_columns = 0;
  for (std::size_t i = 0; i < IMU.size(); ++i)
  {
    imu[i] = column(IMU[i]);
    imu_columns += imu[i] >= 0 ? 1 : 0;
  }
  if (imu_columns != 0 && imu_columns != IMU.size())
  {
    error = "frames.csv has some but not all IMU columns";
    return false;
  }

  rows.clear();
  for (std::size_t line_number = 2; std::getline(file, line); ++line_number)
  {
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    if (line.empty())
    {
      continue;
    }
    const std::vector<std::string> fields = Detail::Split(line);
    std::array<uint64_t, REQUIRED.size()> value{};
    bool ok = fields.size() == header.size();
    for (std::size_t i = 0; ok && i < REQUIRED.size(); ++i)
    {
      ok = Detail::ParseUnsigned(fields[required[i]], value[i]);
    }
    Row row{value[0],
            value[1],
            static_cast<uint32_t>(value[2]),
            {static_cast<uint32_t>(value[3]), static_cast<uint32_t>(value[4]),
             static_cast<uint16_t>(value[5])},
            std::nullopt};
    if (ok && imu_columns != 0)
    {
      std::array<float, IMU.size()> v{};
      for (std::size_t i = 0; ok && i < IMU.size(); ++i)
      {
        ok = Detail::ParseFloat(fields[imu[i]], v[i]);
      }
      row.imu =
          RecordedImu{{v[0], v[1], v[2], v[3]}, {v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
    }
    if (!ok)
    {
      error = "frames.csv line " + std::to_string(line_number) + " is malformed";
      return false;
    }
    rows.push_back(row);
  }
  return true;
}

/**
 * @brief 读一张 640×512 的 8 位 P5 PGM 到 `pixels`。
 *        Read one 640×512 8-bit P5 PGM into `pixels`.
 */
inline bool ReadPgm(const std::string& path, std::span<uint8_t> pixels,
                    std::string& error)
{
  std::ifstream file(path, std::ios::binary);
  std::string magic;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t max_value = 0;
  file >> magic;
  // 跳过注释行 / Skip comment lines.
  const auto next_number = [&file](uint32_t& value)
  {
    file >> std::ws;
    while (file.peek() == '#')
    {
      file.ignore(4096, '\n');
      file >> std::ws;
    }
    return static_cast<bool>(file >> value);
  };
  if (!file || magic != "P5" || !next_number(width) || !next_number(height) ||
      !next_number(max_value))
  {
    error = path + " is not a P5 PGM";
    return false;
  }
  if (width != CameraTypes::FRAME_WIDTH || height != CameraTypes::FRAME_HEIGHT ||
      max_value != 255 || pixels.size() != CameraTypes::FRAME_BYTES)
  {
    error = path + " is " + std::to_string(width) + "x" + std::to_string(height) +
            " max " + std::to_string(max_value) + ", expected 640x512 max 255";
    return false;
  }
  file.get();  // 头部之后的一个空白字符 / The single whitespace after the header
  file.read(reinterpret_cast<char*>(pixels.data()),
            static_cast<std::streamsize>(pixels.size()));
  if (file.gcount() != static_cast<std::streamsize>(pixels.size()))
  {
    error = path + " is truncated";
    return false;
  }
  return true;
}
}  // namespace CaptureFileRecording
