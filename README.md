# CaptureFileCamera

CaptureFileCamera 从视频文件和 IMU CSV 回放相机数据。

它读取一帧图像，读取同序号的一行 IMU 数据，然后写入 `CameraBase<Info>` 的图像槽。

## 输入文件

`file_path` 是视频文件路径。

`imu_csv_path` 是 IMU CSV 路径。每一行对应一帧视频，列顺序为：

```text
timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
```

含义：

- `timestamp_us`：时间戳，单位微秒
- `qw,qx,qy,qz`：四元数，顺序为 `w, x, y, z`
- `gx,gy,gz`：角速度，单位 `rad/s`
- `ax,ay,az`：线加速度，单位 `m/s^2`

视频帧序号和 CSV 有效数据行序号必须一一对应。

## 相机信息

模板参数 `Info` 必须描述视频解码后的图像：

- `encoding` 必须是 `CameraTypes::Encoding::BGR8`
- `step` 必须等于 `width * 3`
- `width` 和 `height` 必须等于视频实际宽高

OpenCV 解码出的灰度图和 BGRA 图会转换成 BGR8。

## 运行参数

`RuntimeParam` 字段：

- `file_path`：视频文件路径
- `imu_csv_path`：IMU CSV 路径
- `camera_name`：相机实例名，也是原始 IMU topic 前缀
- `image_topic_name`：图像名称
- `imu_topic_name`：同步 IMU 名称
- `realtime`：是否按帧间隔限速回放
- `loop`：到达文件末尾后是否从头回放
- `max_frames`：最多提交多少帧，`0` 表示不限制
- `replay_speed`：回放速度倍率，`1.0` 表示原速

测试时可以用环境变量覆盖部分参数：

```text
CAPTURE_FILE_CAMERA_MAX_FRAMES
CAPTURE_FILE_CAMERA_REALTIME
CAPTURE_FILE_CAMERA_REPLAY_SPEED
```

## 回放流程

采集线程流程：

1. 等待图像槽可用
2. 读取一帧视频
3. 读取同序号 IMU 数据
4. 发布 gyro、accl、quat
5. 写入 `ImageFrame::timestamp_us`
6. 写入 `ImageFrame::data`
7. 调用 `CommitImage()`
8. 按 `realtime` 和 `replay_speed` 决定是否等待

先发布 IMU，再提交图像。这样按最新 IMU 取样的同步方式可以拿到同帧数据。

## 输出

图像写入 `CameraBase<Info>` 的图像槽。

原始 IMU topic 名称由 `camera_name` 生成：

```text
<camera_name>_gyro
<camera_name>_accl
<camera_name>_quat
```

## 说明

- 模块不修改图像方向
- 模块不修改相机内参
- 模块不从图像像素中读取元数据
- 视频和 CSV 对不上时，回放结果不可用
