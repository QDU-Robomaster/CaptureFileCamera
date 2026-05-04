#!/usr/bin/env bash
set -euo pipefail

VIDEO_DIR=${VIDEO_DIR:-calib_videos}
RUN_ROOT=${CALIB_RUN_ROOT:-calib_replay_run}
BUILD_DIR=${BUILD_DIR:-build_charuco_replay}
CURRENT_DIR="${RUN_ROOT}/current_input"

export PATH="${HOME}/.local/bin:${PATH}"

mkdir -p "${RUN_ROOT}" "${CURRENT_DIR}" User

make_imu_csv() {
  local video="$1"
  local csv="$2"
  local rate frames period_us

  rate=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=r_frame_rate -of default=nw=1:nk=1 "${video}" | head -1)
  frames=$(ffprobe -v error -count_frames -select_streams v:0 \
    -show_entries stream=nb_read_frames -of default=nw=1:nk=1 "${video}" | head -1)
  if [[ -z "${frames}" || "${frames}" == "N/A" ]]; then
    frames=$(ffprobe -v error -select_streams v:0 \
      -show_entries stream=nb_frames -of default=nw=1:nk=1 "${video}" | head -1)
  fi
  if [[ -z "${rate}" || -z "${frames}" || "${frames}" == "N/A" ]]; then
    echo "无法读取视频帧率或帧数: ${video}" >&2
    return 1
  fi

  period_us=$(python3 - "${rate}" <<'PY'
from fractions import Fraction
import sys
rate = Fraction(sys.argv[1])
if rate <= 0:
    raise SystemExit("invalid frame rate")
print(round(1_000_000 / float(rate)))
PY
)

  python3 - "${frames}" "${period_us}" "${csv}" <<'PY'
import sys

frames = int(sys.argv[1])
period = int(sys.argv[2])
csv_path = sys.argv[3]

with open(csv_path, "w", encoding="utf-8") as out:
    out.write("timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az\n")
    for index in range(frames):
        out.write(f"{index * period},1,0,0,0,0,0,0,0,0,0\n")
PY
}

write_harness() {
  cat > main.cpp <<'EOF'
#include <cstdlib>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <thread>
#include <vector>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_rw.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "xrobot_main.hpp"

namespace
{
const char* EnvOr(const char* name, const char* fallback)
{
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

uint32_t EnvU32Or(const char* name, uint32_t fallback)
{
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
  {
    return fallback;
  }
  char* end = nullptr;
  const auto parsed = std::strtoul(value, &end, 10);
  return end != value && *end == '\0' ? static_cast<uint32_t>(parsed) : fallback;
}

int RunCommand(LibXR::RamFS& ramfs, const std::vector<std::string>& args)
{
  if (args.empty())
  {
    return -1;
  }

  auto* file = ramfs.FindFile(args.front().c_str());
  if (file == nullptr)
  {
    return -2;
  }

  std::vector<char*> argv;
  argv.reserve(args.size());
  for (const auto& arg : args)
  {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  return file->Run(static_cast<int>(argv.size()), argv.data());
}

int WaitAndRunCommand(LibXR::RamFS& ramfs, const std::vector<std::string>& args,
                      uint32_t timeout_ms)
{
  uint32_t waited_ms = 0;
  while (waited_ms <= timeout_ms)
  {
    const int rc = RunCommand(ramfs, args);
    if (rc != -2)
    {
      return rc;
    }
    LibXR::Thread::Sleep(10);
    waited_ms += 10;
  }
  return -2;
}
}  // namespace

int main(int, char**)
{
  LibXR::PlatformInit();
  LibXR::RamFS ramfs;
  LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
  };

  std::thread command_thread([&ramfs]() {
    const std::string camera = EnvOr("CALIB_CAMERA_NAME", "cali_camera");
    const std::string marker = EnvOr("CALIB_MARKER", "25mm");
    const std::string cols = EnvOr("CALIB_COLS", "8");
    const std::string rows = EnvOr("CALIB_ROWS", "6");
    const uint32_t capture_ms = EnvU32Or("CALIB_CAPTURE_MS", 30000);

    const int start_rc =
        WaitAndRunCommand(ramfs, {camera, "cali", marker, cols, rows}, 5000);
    XR_LOG_PASS("charuco replay cali start rc=%d marker=%s board=%sx%s",
                start_rc, marker.c_str(), cols.c_str(), rows.c_str());
    if (start_rc != 0)
    {
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(2);
    }

    LibXR::Thread::Sleep(capture_ms);
    (void)RunCommand(ramfs, {camera, "cali", "status"});
    const int save_rc = RunCommand(ramfs, {camera, "cali", "save"});
    XR_LOG_PASS("charuco replay cali save rc=%d", save_rc);
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(save_rc == 0 ? 0 : 3);
  });
  command_thread.detach();

  XRobotMain(peripherals);
  return 4;
}
EOF

  cat > CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(capturefile_charuco_replay CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(capturefile_charuco_replay main.cpp)
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules/)
add_subdirectory(libxr)
target_include_directories(capturefile_charuco_replay
  PUBLIC
    $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
    ${CMAKE_SOURCE_DIR}/User)
target_link_libraries(capturefile_charuco_replay PUBLIC xr)
EOF
}

write_xrobot_config() {
  local current_video="$1"
  local current_imu="$2"

  mkdir -p User
  cat > User/xrobot.yaml <<EOF
global_settings:
  monitor_sleep_ms: 1000

constexpr_namespace: CharucoReplayConfig

constexpr_includes:
  - CameraBase.hpp
  - CameraFrameSync.hpp

constexprs:
  MainCameraInfo:
    type: CameraTypes::CameraInfo
    value:
      expr: "CameraTypes::CameraInfo{1920, 1080, 5760, CameraTypes::Encoding::BGR8, {1732.0, 0.0, 960.0, 0.0, 1732.0, 540.0, 0.0, 0.0, 1.0}, CameraTypes::DistortionModel::PLUMB_BOB, {0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {1732.0, 0.0, 960.0, 0.0, 0.0, 1732.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0}}"

modules:
  - id: camera
    name: CaptureFileCamera
    template_args:
      Info: {constexpr: MainCameraInfo}
    constructor_args:
      runtime:
        expr: "CaptureFileCamera<CharucoReplayConfig::MainCameraInfo>::RuntimeParam{.file_path = \"${current_video}\", .imu_csv_path = \"${current_imu}\", .camera_name = \"cali_camera\", .image_topic_name = \"cali_image\", .imu_topic_name = \"cali_imu\", .realtime = true, .loop = false, .max_frames = 0}"

  - id: camera_frame_sync
    name: CameraFrameSync
    template_args:
      Info: {constexpr: MainCameraInfo}
    constructor_args:
      camera: '@camera'
      runtime:
        expr: "CameraFrameSync<CharucoReplayConfig::MainCameraInfo>::RuntimeParam{.mode = CameraFrameSync<CharucoReplayConfig::MainCameraInfo>::SyncMode::LATEST_IMU, .offset_us = 0}"
EOF
}

parse_calibration_yaml() {
  local yaml="$1"
  local min_views="$2"
  local max_rms="$3"
  python3 - "${yaml}" "${min_views}" "${max_rms}" <<'PY'
import re
import sys

yaml_path, min_views_text, max_rms_text = sys.argv[1:4]
text = open(yaml_path, encoding="utf-8").read()
views_match = re.search(r"^views:\s*([0-9]+)", text, re.M)
rms_match = re.search(r"^rms:\s*([-+0-9.eE]+)", text, re.M)
k_match = re.search(r"camera_matrix:.*?data:\s*\[([^\]]+)\]", text, re.S)
if not (views_match and rms_match and k_match):
    raise SystemExit("calibration yaml missing views/rms/camera_matrix")

views = int(views_match.group(1))
rms = float(rms_match.group(1))
k = [float(x) for x in re.split(r"[\s,]+", k_match.group(1).strip()) if x]
min_views = int(min_views_text)
max_rms = float(max_rms_text)
if views < min_views:
    raise SystemExit(f"views too low: {views} < {min_views}")
if rms > max_rms:
    raise SystemExit(f"rms too high: {rms:.6f} > {max_rms:.6f}")
print(f"views={views} rms={rms:.6f} fx={k[0]:.6f} fy={k[4]:.6f} cx={k[2]:.6f} cy={k[5]:.6f}")
PY
}

check_quality_report() {
  local report="$1"

  if [[ ! -f "${report}" ]]; then
    echo "缺少标定质量报告: ${report}" >&2
    return 1
  fi
  if ! grep -q "质量总判定(quality_ok): 通过" "${report}"; then
    echo "标定质量总判定未通过: ${report}" >&2
    return 1
  fi
  if ! grep -q "重投影判定(reprojection_ok): 通过" "${report}"; then
    echo "重投影判定未通过: ${report}" >&2
    return 1
  fi
}

run_case() {
  local name="$1"
  local marker="$2"
  local cols="$3"
  local rows="$4"
  local capture_ms="$5"
  local min_views="$6"
  local max_rms="$7"

  local video="${VIDEO_DIR}/${name}.mp4"
  local case_dir="${RUN_ROOT}/${name}"
  mkdir -p "${case_dir}"
  if [[ ! -f "${video}" ]]; then
    echo "缺少测试视频: ${video}" >&2
    return 1
  fi

  make_imu_csv "${video}" "${case_dir}/${name}_imu.csv"
  ln -sf "$(realpath "${video}")" "${CURRENT_DIR}/current.mp4"
  ln -sf "$(realpath "${case_dir}/${name}_imu.csv")" "${CURRENT_DIR}/current_imu.csv"

  rm -rf runs/camera_calib
  CALIB_MARKER="${marker}" CALIB_COLS="${cols}" CALIB_ROWS="${rows}" \
    CALIB_CAPTURE_MS="${capture_ms}" \
    timeout "$((capture_ms / 1000 + 25))s" \
    "./${BUILD_DIR}/capturefile_charuco_replay" \
    > "${case_dir}/run.log" 2>&1

  local yaml
  yaml=$(find runs/camera_calib -name calibration.yml -print 2>/dev/null | sort | tail -1 || true)
  if [[ -z "${yaml}" ]]; then
    echo "${name} FAIL no_calibration_yaml" | tee -a "${RUN_ROOT}/summary.txt"
    tail -120 "${case_dir}/run.log" >&2 || true
    return 1
  fi
  check_quality_report "$(dirname "${yaml}")/quality_report.txt"

  mkdir -p "${case_dir}/calibration"
  cp -a "$(dirname "${yaml}")"/. "${case_dir}/calibration/"

  local result
  result=$(parse_calibration_yaml "${yaml}" "${min_views}" "${max_rms}")
  echo "${name} PASS ${result} yaml=${case_dir}/calibration/calibration.yml" \
    | tee -a "${RUN_ROOT}/summary.txt"
}

write_harness
write_xrobot_config "$(realpath "${CURRENT_DIR}/current.mp4")" \
  "$(realpath "${CURRENT_DIR}/current_imu.csv")"

xrobot_gen_main --config User/xrobot.yaml --output User/xrobot_main.hpp
cmake -S . -B "${BUILD_DIR}" -G Ninja
cmake --build "${BUILD_DIR}" -j"$(nproc)"

{
  echo "video_dir=$(realpath "${VIDEO_DIR}")"
  echo "run_root=$(realpath "${RUN_ROOT}")"
  echo "opencv=$(pkg-config --modversion opencv4 2>/dev/null || true)"
} > "${RUN_ROOT}/summary.txt"

run_case "25mm_4x4" "25mm" 4 4 28500 50 3.0
run_case "25mm_8x6" "25mm" 8 6 26000 40 3.0
run_case "15mm_11x8" "15mm" 11 8 31000 50 3.0

cat "${RUN_ROOT}/summary.txt"
