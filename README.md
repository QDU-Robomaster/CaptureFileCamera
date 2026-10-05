# CaptureFileCamera

回放相机：按录制时间回放统一录像，并在 MCU 的 IMU Topic 上发布录制的 IMU / Replay camera that plays a unified recording at its recorded pace and publishes the recorded IMU on the MCU IMU Topics

## 1. 模块作用 / Purpose

CaptureFileCamera 是 CameraBase 的回放驱动。它读入一段录像，在每帧的录制时刻先发布这一帧的 IMU，再发布图像，下游的 CameraFrameSync、检测器、跟踪器与实车运行时相同。回放只用于离线调试和回归，录像来自车上的录制器或台架采集工具 rawcap。

CaptureFileCamera is the CameraBase replay driver. It reads a recording and, at each frame's recorded time, publishes that frame's IMU and then the image, so CameraFrameSync, the detector and the tracker run as they do on the robot. Replay serves offline debugging and regression; recordings come from the on-robot recorder or the bench capture tool rawcap.

## 2. 录像格式 / Recording Format

一个录像是一个目录：

A recording is a directory:

```text
session.txt   录制时的相机设置（回放不读）/ camera settings (not read by replay)
frames.csv    每帧一行 / one row per frame
000000.pgm    640×512 P5 8 位 BayerRG8，(0, 0) 为 R / 8-bit BayerRG8, R at (0, 0)
000001.pgm
...
```

`frames.csv` 的列按表头名查找：

The columns of `frames.csv` are found by header name:

| 列 / Column | 含义 / Meaning |
| --- | --- |
| `frame` | 文件序号，图像为 `<frame>.pgm`（6 位补零）/ File index, image `<frame>.pgm` (6 digits) |
| `timestamp_us` | 采样时间，微秒，不递减 / Sampling time in µs, non-decreasing |
| `frame_counter` | 相机帧计数 / Camera frame counter |
| `roi_x`, `roi_y`, `decimation` | 这一帧的几何（原生坐标）/ Geometry of the frame, native coordinates |
| `qw,qx,qy,qz,gx,gy,gz,ax,ay,az` | 可选：姿态四元数、角速度 rad/s、加速度 m/s² / Optional IMU |

IMU 十列要么都有要么都没有，坐标系为机体系 x 右、y 前、z 上。车上录制器写入同步后的 IMU；台架录像没有 IMU 列，回放时发布静止姿态（单位四元数、零角速度、`az = 9.80665`），跟踪结果按云台不动解释。

The ten IMU columns are all present or all absent, in the body frame x right, y forward, z up. The on-robot recorder writes the synced IMU; bench recordings have no IMU columns, and replay then publishes a resting attitude (identity quaternion, zero angular velocity, `az = 9.80665`), so tracking results read as a still gimbal.

```text
frame,timestamp_us,frame_counter,roi_x,roi_y,decimation,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
0,1000,10,80,24,2,1,0,0,0,0,0,0,0,0,9.8
1,11000,11,80,24,2,1,0,0,0,0,0,0.01,0,0,9.8
```

构造时读完 `frames.csv` 并检查每一行：几何在传感器内、时间戳不递减、PGM 文件存在。任何一项不满足即致命退出并打印原因。

At construction `frames.csv` is read and every row is checked: geometry inside the sensor, non-decreasing timestamps and an existing PGM file. Any failure is fatal and logged.

## 3. 回放 / Replay

`ReplaySettings` 与 YAML 一一对应：

`ReplaySettings` maps one-to-one to the YAML:

| 字段 / Field | 含义 / Meaning |
| --- | --- |
| `recording_dir` | 录像目录 / Recording directory |
| `gyro_topic`, `accl_topic`, `quat_topic` | IMU Topic 名，与 CameraFrameSync 的配置相同 / IMU Topic names, as configured in CameraFrameSync |
| `speed` | 1.0 为录制速度，2.0 为两倍速，0 为不限速 / 1.0 = recorded pace, 0 = unpaced |
| `loop` | 播完从头开始 / Restart at the end |
| `max_frames` | 发布这么多帧后停止，0 为不限 / Stop after this many frames, 0 = unlimited |

每帧先在三个 IMU Topic 上发布 IMU（Topic 时间戳为帧时间），再发布图像；图像的时间戳、帧计数、几何都取自录像。三个 IMU Topic 由本模块创建，载荷与 MCU 相同：角速度、加速度为 `Eigen::Matrix<float, 3, 1>`，姿态为 `LibXR::Quaternion<float>`。

Each frame publishes the IMU on the three IMU Topics first (Topic timestamp = frame time) and then the image; the image timestamp, frame counter and geometry come from the recording. This Module creates the three IMU Topics with the MCU payloads: `Eigen::Matrix<float, 3, 1>` for angular velocity and acceleration, `LibXR::Quaternion<float>` for the attitude.

没有空图像槽时回放等待，不丢帧，同一录像的结果可以复现。循环播放时每轮时间戳平移一轮的时长（首尾间隔加一个平均帧间隔），保持递增。播完后打印 `replay finished after N frames`，相机停止发布。几何由录像决定，`SwitchView` 的请求被接受但不生效。

With no free image slot replay waits instead of dropping, so results on one recording are reproducible. When looping, each pass shifts the timestamps by one pass length (first-to-last span plus one mean frame period) so they keep increasing. At the end it logs `replay finished after N frames` and stops publishing. The geometry follows the recording; `SwitchView` requests are accepted and have no effect.

## 4. 配置示例 / Configuration Example

```yaml
modules:
  - module: QDU-Robomaster/CaptureFileCamera
    id: camera
    args:
      - calibration: ReplayConfig::MainCameraCalibration
      - name: "gimbal"
      - settings:
          recording_dir: "recordings/wide_20261004"
          gyro_topic: "gimbal_gyro"
          accl_topic: "gimbal_accl"
          quat_topic: "gimbal_quat"
          speed: 1.0
          loop: false
          max_frames: 0
```

回放配置中 CameraFrameSync 使用 `LATEST_IMU` 模式，IMU Topic 名与这里相同。

In a replay configuration CameraFrameSync runs in `LATEST_IMU` mode with the same IMU Topic names.

## 5. 测试 / Tests

- `tests/recording_test.cpp`：`frames.csv` 按表头读列、IMU 列可选、缺列与格式错误；PGM 读取、注释行、尺寸错误与截断。
- `tests/capture_file_camera_test.cpp`：每帧 IMU 先于图像且时间相同、几何与帧计数取自录像、循环时间递增与 `max_frames`、无 IMU 时的静止姿态、按录制速度限速。

- `tests/recording_test.cpp`: `frames.csv` columns by header name, optional IMU columns, missing columns and malformed rows; PGM reading, comment lines, wrong sizes and truncation.
- `tests/capture_file_camera_test.cpp`: IMU before each image with the same time, geometry and frame counter from the recording, increasing time when looping and `max_frames`, the resting attitude without IMU, and pacing at the recorded rate.

## 6. 依赖 / Dependencies

CameraBase、LibXR（Topic、Eigen、Quaternion）。

CameraBase, LibXR (Topic, Eigen, Quaternion).
