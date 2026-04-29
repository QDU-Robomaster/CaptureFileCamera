# CaptureFileCamera

独立文件相机模块，用清洗后的 capture package 复现真实相机输入链路。

输入文件：

- `file_path`：干净视频文件，只包含图像数据。
- `imu_csv_path`：逐帧 IMU CSV，列顺序为 `timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az`。

输出：

- 图像：`BGR8 1440x1080 step=4320`
- 原始 IMU 话题：`<camera_name>_gyro`、`<camera_name>_accl`、`<camera_name>_quat`

要求：

- 视频帧序号和 IMU CSV 行序号一一对应。
- 模块不从图像像素里解析私有元数据。
- 文件源应配合 `CameraFrameSync::LATEST_IMU` 使用，不模拟 Hik 实机的硬件触发延迟。
