#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "CaptureFileRecording.hpp"

namespace
{
namespace fs = std::filesystem;

void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

std::string MakeDir(const char* name)
{
  const fs::path dir = fs::temp_directory_path() / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir.string();
}

void WriteText(const std::string& path, const std::string& text)
{
  std::ofstream(path, std::ios::binary) << text;
}

void TestFramesWithImu()
{
  const std::string dir = MakeDir("cfc_recording_imu");
  WriteText(
      dir + "/frames.csv",
      "frame,timestamp_us,frame_counter,roi_x,roi_y,decimation,qw,qx,qy,qz,gx,gy,gz,"
      "ax,ay,az\r\n"
      "7,1000,3,80,24,2,1,0,0,0,0.5,0,0,0,0,9.8\r\n"
      "8,11000,4,400,284,1,0.7071,0,0,0.7071,0,0,-1.25,0.1,0,9.8\r\n");
  std::vector<CaptureFileRecording::Row> rows;
  std::string error;
  Expect(CaptureFileRecording::LoadFrames(dir, rows, error), error.c_str());
  Expect(rows.size() == 2, "two rows");
  Expect(rows[0].frame == 7 && rows[0].timestamp_us == 1000 && rows[0].frame_counter == 3,
         "row 0 fields");
  Expect(rows[0].geometry == CameraTypes::FrameGeometry{80, 24, 2}, "row 0 geometry");
  Expect(rows[1].geometry == CameraTypes::FrameGeometry{400, 284, 1}, "row 1 geometry");
  Expect(rows[1].imu && rows[1].imu->angular_velocity_xyz[2] == -1.25F, "row 1 gyro z");
  Expect(rows[1].imu->rotation_wxyz[3] == 0.7071F, "row 1 quaternion z");
  Expect(CaptureFileRecording::PgmPath(dir, 7) == dir + "/000007.pgm", "PGM name");
}

void TestFramesWithoutImu()
{
  const std::string dir = MakeDir("cfc_recording_plain");
  // 列顺序按表头 / Columns are found by header name.
  WriteText(dir + "/frames.csv",
            "decimation,roi_y,roi_x,frame_counter,timestamp_us,frame\n2,24,80,0,500,0\n");
  std::vector<CaptureFileRecording::Row> rows;
  std::string error;
  Expect(CaptureFileRecording::LoadFrames(dir, rows, error), error.c_str());
  Expect(rows.size() == 1 && !rows[0].imu, "no IMU");
  Expect(rows[0].timestamp_us == 500 && rows[0].geometry.decimation == 2, "reordered");
}

void TestBadFrames()
{
  const std::string dir = MakeDir("cfc_recording_bad");
  std::vector<CaptureFileRecording::Row> rows;
  std::string error;
  Expect(!CaptureFileRecording::LoadFrames(dir, rows, error), "missing file");
  WriteText(dir + "/frames.csv", "frame,timestamp_us,roi_x,roi_y,decimation\n");
  Expect(!CaptureFileRecording::LoadFrames(dir, rows, error), "missing column");
  WriteText(
      dir + "/frames.csv",
      "frame,timestamp_us,frame_counter,roi_x,roi_y,decimation,qw\n0,0,0,80,24,2,1\n");
  Expect(!CaptureFileRecording::LoadFrames(dir, rows, error), "partial IMU");
  WriteText(dir + "/frames.csv",
            "frame,timestamp_us,frame_counter,roi_x,roi_y,decimation\n0,x,0,80,24,2\n");
  Expect(!CaptureFileRecording::LoadFrames(dir, rows, error), "malformed number");
  Expect(error.find("line 2") != std::string::npos, "error names the line");
}

void TestPgm()
{
  const std::string dir = MakeDir("cfc_recording_pgm");
  std::string pixels(CameraTypes::FRAME_BYTES, '\0');
  pixels[0] = 42;
  pixels.back() = 7;
  WriteText(dir + "/a.pgm", "P5\n# comment\n640 512\n255\n" + pixels);
  std::vector<uint8_t> out(CameraTypes::FRAME_BYTES);
  std::string error;
  Expect(CaptureFileRecording::ReadPgm(dir + "/a.pgm", out, error), error.c_str());
  Expect(out[0] == 42 && out.back() == 7, "PGM bytes");

  WriteText(dir + "/b.pgm", "P5\n320 256\n255\n" + pixels.substr(0, 320 * 256));
  Expect(!CaptureFileRecording::ReadPgm(dir + "/b.pgm", out, error), "wrong size");
  WriteText(dir + "/c.pgm", "P5\n640 512\n255\n" + pixels.substr(0, 1000));
  Expect(!CaptureFileRecording::ReadPgm(dir + "/c.pgm", out, error), "truncated");
  WriteText(dir + "/d.pgm", "P6\n640 512\n255\n" + pixels);
  Expect(!CaptureFileRecording::ReadPgm(dir + "/d.pgm", out, error), "not P5");
}
}  // namespace

int main()
{
  TestFramesWithImu();
  TestFramesWithoutImu();
  TestBadFrames();
  TestPgm();
  std::puts("capture_file_recording_test passed");
  return 0;
}
