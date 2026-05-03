# CaptureFileCamera

`CaptureFileCamera` 是内录文件相机模块，用一份清洗后的 capture package
复现相机输入。它发布原始图像和原始 IMU 话题，后级仍然通过
`CameraFrameSync` 产出同步后的 `camera_imu`。

这个模块用于回放调试，不模拟 Hik 实机的硬件触发延迟，也不从图像像素里解析私有元数据。

## 输入文件

运行时需要显式传入两份文件：

- `file_path`：干净视频文件，只包含图像。
- `imu_csv_path`：与视频帧一一对应的 IMU CSV。

默认路径只是占位值：

- `capture.avi`
- `capture_imu.csv`

BSP 或测试配置应按实际数据包路径覆盖它们。

## 视频格式

视频解码后会统一转换为 BGR8，再写入 `CameraBase` 图像槽位。
CSV 解析和视频转换的格式处理集中在 `capture_file_package.hpp`，主模块只负责发布。

当前要求：

- 分辨率必须等于模板参数里的 `CameraInfo::width / height`。
- 行跨度必须满足紧密 BGR8：`step = width * 3`。
- 支持 OpenCV 解码出的 `CV_8UC3`、`CV_8UC4` 和 `CV_8UC1`。

如果 `realtime = true`，模块按相邻 CSV 时间戳差值限速播放；
缺少有效相邻差值时，使用视频 FPS 推导的周期。这里的限速只控制回放节奏，
不参与同步判定。

## IMU CSV 格式

每一行对应一帧视频。列顺序固定为：

```text
timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
```

字段含义：

- `timestamp_us`：传感器侧采样时间戳，单位微秒。
- `qw,qx,qy,qz`：姿态四元数，顺序为 `wxyz`。
- `gx,gy,gz`：角速度，单位 `rad/s`。
- `ax,ay,az`：线加速度，单位 `m/s^2`。

CSV 可以包含空行、以 `#` 开头的注释行，以及一行表头。第一条有效数据之后，
任何无法解析的行都会被视为数据错误并中止初始化。

## 输出话题

图像通过 `CameraBase` 的共享图像槽位发布：

- `image_topic_name`：默认 `camera_image`

原始 IMU 通过普通 Topic 发布，Topic timestamp 使用 CSV 中的 `timestamp_us`。
payload 只包含测量值：

- `<camera_name>_gyro`：`std::array<float, 3>`
- `<camera_name>_accl`：`std::array<float, 3>`
- `<camera_name>_quat`：`std::array<float, 4>`

同步后的 `imu_topic_name` 由 `CameraFrameSync` 发布，本模块不直接发布同步 IMU。

## 手动标定回归

本仓库的 `Manual ChArUco Calibration Replay` GitHub Actions workflow 用三段
ChArUco 录制视频回放 `CaptureFileCamera`，并调用 `CameraBase` 的 `cali`
命令保存标定结果。标定算法仍归属 `CameraBase`；这里验证内录文件相机能稳定复现这条输入链路。

默认视频资产来自仓库 release `charuco-calib-videos-20260504`。手动触发 workflow
时可以覆盖 `video_base_url`、`CameraBase` / `CameraFrameSync` / `CameraSync`
以及 `libxr` 的依赖 ref。
