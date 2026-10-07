# CaptureFileCamera

回放相机：按录制时间回放统一录像，直接发布图像与同步帧 / Replay camera that plays a unified recording at its recorded pace and publishes the images and the synced frames directly

## 1. 模块作用 / Purpose

CaptureFileCamera 是 CameraBase 的回放驱动。它读入一段录像，在每帧的录制时刻发布图像 `<name>_image` 和同步帧 `<name>_synced`。录像里每帧的 IMU 就是录制时同步帧里的 IMU，所以回放直接发同步帧，不经过 CameraFrameSync，检测器和跟踪器收到的输入与录制时相同。回放只用于离线调试和回归，录像来自车上的录制器或台架采集工具 rawcap。

CaptureFileCamera is the CameraBase replay driver. It reads a recording and, at each frame's recorded time, publishes the image `<name>_image` and the synced frame `<name>_synced`. The IMU recorded with each frame is the IMU of the synced frame at recording time, so replay publishes synced frames directly without CameraFrameSync, and the detector and tracker receive the same input as when the recording was made. Replay serves offline debugging and regression; recordings come from the on-robot recorder or the bench capture tool rawcap.

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

IMU 十列要么都有要么都没有，坐标系为机体系 x 右、y 前、z 上。车上录制器写入同步帧里的 IMU；台架录像没有 IMU 列，回放时同步帧里放静止姿态（单位四元数、零角速度、`az = 9.80665`），跟踪结果按云台不动解释。

The ten IMU columns are all present or all absent, in the body frame x right, y forward, z up. The on-robot recorder writes the IMU of each synced frame; bench recordings have no IMU columns, and replay then puts a resting attitude (identity quaternion, zero angular velocity, `az = 9.80665`) in the synced frame, so tracking results read as a still gimbal.

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
| `speed` | 1.0 为录制速度，2.0 为两倍速，0 为不限速 / 1.0 = recorded pace, 0 = unpaced |
| `loop` | 播完从头开始 / Restart at the end |
| `max_frames` | 发布这么多帧后停止，0 为不限 / Stop after this many frames, 0 = unlimited |

每帧先发布图像，再用同一张图发布 `SyncedFrame{sequence, image, imu}`；图像的时间戳、帧计数、几何和 IMU 都取自录像，IMU 的时间戳等于帧时间，`sequence` 从 1 起逐帧加一。两个 Topic 都由本模块创建。

Each frame publishes the image and then `SyncedFrame{sequence, image, imu}` holding the same image. The timestamp, frame counter, geometry and IMU come from the recording; the IMU timestamp equals the frame time and `sequence` starts at 1 and increases by one per frame. This Module creates both Topics.

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
          speed: 1.0
          loop: false
          max_frames: 0
```

回放配置里不放 CameraFrameSync；检测器订阅 `gimbal_synced`，与实车相同。

A replay configuration has no CameraFrameSync; the detector subscribes to `gimbal_synced` as on the robot.

## 5. 测试 / Tests

- `tests/recording_test.cpp`：`frames.csv` 按表头读列、IMU 列可选、缺列与格式错误；PGM 读取、注释行、尺寸错误与截断。
- `tests/capture_file_camera_test.cpp`：每帧先发图像再发持有同一张图的同步帧、IMU 与时间戳、几何与帧计数取自录像、循环时间递增与 `max_frames`、无 IMU 时的静止姿态、按录制速度限速。

- `tests/recording_test.cpp`: `frames.csv` columns by header name, optional IMU columns, missing columns and malformed rows; PGM reading, comment lines, wrong sizes and truncation.
- `tests/capture_file_camera_test.cpp`: the image followed by a synced frame holding the same image, IMU and timestamps, geometry and frame counter from the recording, increasing time when looping and `max_frames`, the resting attitude without IMU, and pacing at the recorded rate.

## 6. 依赖 / Dependencies

CameraBase、AutoAimTypes、LibXR。

CameraBase, AutoAimTypes, LibXR.
