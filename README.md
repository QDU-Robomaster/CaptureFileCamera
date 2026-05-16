# CaptureFileCamera

`CaptureFileCamera` 是文件回放相机。它从录制文件读取图像和 IMU 数据，按原始
timestamp 发布给 `CameraFrameSync`，用于复现实机相机输入、调试同步、回归视觉流程。

模块支持两种输入：

- 普通视频 + IMU CSV。
- 帧索引 CSV + 帧数据 bin + IMU CSV。

它不模拟 Hik 相机触发延迟，也不从图像像素中读取额外信息。

## 运行参数

`runtime` 参数由 BSP YAML 传入：

- `file_path`：普通视频路径，或者帧数据 bin 路径。
- `imu_csv_path`：IMU CSV 路径。
- `frame_csv_path`：帧索引 CSV 路径；为空时按普通视频回放。
- `camera_name`：相机名，也是原始 IMU 话题前缀。
- `image_topic_name`：图像话题名，默认 `camera_image`。
- `imu_topic_name`：同步后 IMU 话题名，默认 `camera_imu`。
- `realtime`：为 `true` 时按录制 timestamp 控制回放速度。
- `loop`：为 `true` 时播放到文件末尾后重新开始。
- `max_frames`：最大提交帧数，`0` 表示不限制。

测试环境可以用环境变量覆盖部分参数：

- `CAPTURE_FILE_CAMERA_MAX_FRAMES`：限制本次回放提交的图像帧数。
- `CAPTURE_FILE_CAMERA_REALTIME=0`：关闭实时限速。

## 普通视频

普通视频模式使用 `file_path` 和 `imu_csv_path`。视频只保存图像，IMU CSV 每一行对应
一帧图像。模块按帧序号同时发布一组原始 IMU 和一帧图像。

视频解码后会转换为 BGR8，再写入 `CameraBase` 的图像缓冲区。当前要求：

- 分辨率等于模板参数 `CameraInfo::width / height`。
- `CameraInfo::encoding` 为 `BGR8`。
- `CameraInfo::step == CameraInfo::width * 3`，即每行是紧密排列的 BGR8 字节。
- OpenCV 解码结果可以是 `CV_8UC3`、`CV_8UC4` 或 `CV_8UC1`。

如果 `realtime = true`，相邻 IMU timestamp 的差值用于控制回放间隔。无法得到有效
差值时，使用视频 FPS 推导间隔。这个限速只影响播放速度，不改变发布 timestamp。

## 帧数据 Bin

`frame_csv_path` 非空时使用帧数据 bin 模式：

- `file_path` 指向 `*_frames.bin`。
- `frame_csv_path` 指向 `*_frames.csv`。
- `imu_csv_path` 指向 `*_imu.csv`。

帧索引 CSV 列顺序为：

```text
frame_index,camera_timestamp_us,offset_bytes,size_bytes[,codec]
```

`offset_bytes` 和 `size_bytes` 指向 bin 文件中的一帧图像。`codec` 可选：

- `png`：PNG 无损图像。
- `jpg` / `jpeg`：JPEG 图像。
- `raw`：未压缩 BGR8 字节，大小必须等于 `CameraBase::image_bytes`。
- 空：按大小判断；等于 `CameraBase::image_bytes` 时按 `raw`，否则交给 OpenCV 解码。

模块用 `camera_timestamp_us` 和 IMU CSV 的 `timestamp_us` 对齐，只播放能找到同 timestamp
IMU 的图像帧。CSV 最后一行如果因为录制结束被截断，会被忽略；中间行解析失败会直接报错。

## IMU CSV

IMU CSV 列顺序为：

```text
timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
```

字段含义：

- `timestamp_us`：传感器时间戳，单位微秒。
- `qw,qx,qy,qz`：姿态四元数，顺序为 `wxyz`。
- `gx,gy,gz`：角速度，单位 `rad/s`。
- `ax,ay,az`：线加速度，单位 `m/s^2`。

CSV 可以包含空行、以 `#` 开头的注释行，以及一行表头。第一条有效数据之后，无法解析的
行会让模块停止初始化。

## 输出

图像写入 `CameraBase` 图像缓冲区，并发布到 `image_topic_name`。

原始 IMU 按 `camera_name` 生成三个话题，Topic timestamp 使用 CSV 的 `timestamp_us`：

- `<camera_name>_gyro`：`Eigen::Matrix<float, 3, 1>`。
- `<camera_name>_accl`：`Eigen::Matrix<float, 3, 1>`。
- `<camera_name>_quat`：`LibXR::Quaternion<float>`。

同步后的 `imu_topic_name` 由 `CameraFrameSync` 发布，`CaptureFileCamera` 只发布原始
IMU。

## 典型用途

- 用实机录制文件复现相机输入。
- 调试 `CameraFrameSync` 的图像和 IMU 对齐。
- 在 CI 或本地回放固定数据，检查检测、跟踪、预览等模块是否还能稳定运行。
