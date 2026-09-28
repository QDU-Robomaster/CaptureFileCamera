# CaptureFileCamera

`CaptureFileCamera` 是文件回放相机，主要使用统一内录包：

- `frames.bin`
- `frames.csv`
- `imu.csv`

`frame_csv_path` 为空时，`file_path` 也可指向历史视频文件，与 IMU CSV 按帧序号配对。

图像按 `FrameLayoutV` 的 BGR8 布局写入 CameraBase 并发布，同时发布原始 IMU，用于复现
实机相机输入、调试 `CameraFrameSync` 同步、回归视觉流程。它不模拟 Hik 相机触发延迟，
也不从图像像素中读取额外信息。

## 相机几何

- 模板参数 `FrameLayoutV` 固定帧缓冲区宽高、步长和编码；必须是紧密排列的 BGR8
  （`step == width * 3`，编译期检查）。
- 构造参数 `calibration` 是原生传感器坐标系下的固定 `CameraCalibration`。
- `runtime.geometry` 是本次回放固定使用的 `FrameGeometry`，会按值复制到每个
  `ImageFrame`。

构造时会验证 geometry 的尺寸、步长、下采样、ROI 和原生边界。内录包的像素尺寸必须与
`FrameLayoutV` 一致。默认值对应 wide 回放：原生 `1440x1080` 标定，`720x540` 布局，
ROI 偏移 0、2x2 下采样几何。

文件相机只公布一个固定档位（`WIDE`，周期为 `trigger_period_us`）。请求该档位直接返回
其逐帧 geometry；请求其他档位返回 `NOT_SUPPORT`，不会改变回放状态。

## 回放节奏

构造时加载并校验 IMU CSV、帧索引 CSV 和帧数据 bin（或打开视频），任何错误都会抛出
异常；随后启动一个后台线程回放。

`realtime: true` 时，每轮以第一帧为计时起点，按录制相对时间除以 `replay_speed`，在发布
对应 IMU 和图像前等待。循环播放时重新计时，录制时间戳仍回到文件起点。解码或下游处理
较慢时不为追赶倍率丢帧。等待使用毫秒级时钟；无法表示的目标时间会记录错误并停止回放。
`realtime: false` 时不限速。倍率只改变播放节奏，不修改图像、IMU 的录制时间戳或档位
触发周期。

每帧先发布对应的原始 IMU，再写入并提交图像。文件回放要求输入无损：CameraBase 图像池
暂时没有空槽时，回放线程保留当前已解码帧并每 1 ms 重试。等待期间不会重复发布该帧 IMU，
也不会推进输入索引；析构时的停止请求可以打断等待。这是文件源的背压策略，不代表实时相机
能在外部触发持续运行时保留所有物理帧。

读取或解码失败、帧尺寸不符时停止回放。到达文件末尾时，`loop: false` 停止，`loop: true`
从头开始；达到 `max_frames` 时停止。

测试环境可以用环境变量覆盖部分参数：

- `CAPTURE_FILE_CAMERA_MAX_FRAMES`：限制本次回放提交的图像帧数（大于 0 时生效）。
- `CAPTURE_FILE_CAMERA_REALTIME=0`：关闭实时限速。

回放倍率仅通过配置 `replay_speed` 设置。

## 帧数据 Bin

统一内录包使用：

- `file_path` 指向 `*_frames.bin`。
- `frame_csv_path` 指向 `*_frames.csv`。
- `imu_csv_path` 指向 `*_imu.csv`。

帧索引 CSV 列顺序为：

```text
frame_index,camera_timestamp_us,offset_bytes,size_bytes[,codec]
```

`offset_bytes` 和 `size_bytes` 指向 bin 文件中的一帧图像，构造时检查其落在文件范围内。
`codec` 为 `raw`（不区分大小写），或缺省且 `size_bytes == CameraBase::image_bytes` 时，
该帧按未压缩 BGR8 读取，大小必须等于 `image_bytes`；其他记录交给 OpenCV `imdecode`
解码（8 位灰度、BGR、BGRA 统一转为 BGR8），解码结果尺寸必须与布局一致。

模块用 `camera_timestamp_us` 和 IMU CSV 的 `timestamp_us` 对齐，只播放能找到同 timestamp
IMU 的图像帧，其余帧跳过（启动日志给出跳过数量）。没有任何对齐帧时构造失败。图像时间戳
为 `camera_timestamp_us`。

两种 CSV 都可以包含空行、以 `#` 开头的注释行，以及第一条数据前的一行表头。最后一行
如果因为录制结束被截断，会被忽略；其他无法解析的行会让构造失败。

## 历史视频

`frame_csv_path` 为空时，`file_path` 由 OpenCV 按视频打开，第 N 帧与 IMU CSV 第 N 行配对，
图像时间戳取该 IMU 行的 `timestamp_us`。视频尺寸必须与布局一致，解码结果统一转为 BGR8。

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

## 输出

图像写入 `CameraBase` 图像缓冲区，并发布到 `image_topic_name`。

原始 IMU 按 `camera_name` 生成三个默认 domain（`libxr_def_domain`）中的话题，Topic
timestamp 使用 CSV 的 `timestamp_us`：

- `<camera_name>_gyro`：`Eigen::Matrix<float, 3, 1>`。
- `<camera_name>_accl`：`Eigen::Matrix<float, 3, 1>`。
- `<camera_name>_quat`：`LibXR::Quaternion<float>`。

`CaptureFileCamera` 只发布原始 IMU，不发布同步结果。回放包中没有 CameraSync 触发事件，
与 CameraFrameSync 配合时通常使用 `LATEST_IMU` 模式，并把其 `host_topic_domain_name`
设为 `"libxr_def_domain"`。

`SetExposure()` / `SetGain()` 为空操作，CameraBase 的 RamFS 命令对文件相机无效果。

`OnMonitor()` 打印累计提交帧数、已发布 IMU 组数、运行状态和本周期提交帧数，以及读帧、
BGR 转换、IMU 发布、图像提交和限速等待的耗时统计（微秒）。

## 依赖

- `QDU-Robomaster/CameraBase`：相机基类与共享图像槽。
- `xrobot-org/DurationStatistics`：`OnMonitor()` 中的耗时统计。
- 外部：OpenCV 4（`core`、`imgproc`、`imgcodecs`、`videoio`），Eigen。

## 构造接口

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CaptureFileCamera : public CameraBase<FrameLayoutV>;

explicit CaptureFileCamera(
    LibXR::RamFS& ramfs,
    CameraCalibration calibration = DefaultCalibration(),
    RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：帧布局，紧密排列 BGR8，必须与回放数据的图像尺寸一致。

依赖：

- `ramfs`：`LibXR::RamFS`，注册 CameraBase 的相机命令文件。

配置：

- `calibration`：原生标定。默认 `DefaultCalibration()` 为 1440x1080、
  `fx ≈ fy ≈ 2328.7`、`PLUMB_BOB` 五项畸变的实机标定。
- `runtime`（`RuntimeParam`）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `file_path` | `"capture_frames.bin"` | 帧数据 bin 路径；`frame_csv_path` 为空时为视频路径。 |
| `frame_csv_path` | `"capture_frames.csv"` | 帧索引 CSV 路径；为空时使用视频回放。 |
| `imu_csv_path` | `"capture_imu.csv"` | IMU CSV 路径。 |
| `camera_name` | `"camera"` | 相机名、原始 IMU 话题前缀和 RamFS 命令文件名。 |
| `image_topic_name` | `"camera_image"` | 图像话题名。 |
| `imu_topic_name` | `"camera_imu"` | 同步 IMU 话题名，交给 CameraBase。 |
| `realtime` | `true` | 按录制时间戳控制回放速度。 |
| `loop` | `false` | 播放到末尾后重新开始。 |
| `max_frames` | `0` | 最大提交帧数，`0` 表示不限制。 |
| `trigger_period_us` | `10000` | 单档对应的图像触发周期，单位 us，必须非零。 |
| `geometry` | 布局尺寸、ROI 0、2x2 下采样 | 帧坐标到原生传感器坐标的固定映射。 |
| `replay_speed` | `1.0` | 回放倍率，必须是有限正数；`0.5` 为半速，`2.0` 为两倍速。 |

`RuntimeParam` 还保留一个不含 `trigger_period_us`（`geometry` 紧跟 `max_frames`）的构造函数，
此时周期取 `10000`。

## 使用

```sh
xrobot module add QDU-Robomaster/CaptureFileCamera
xrobot setup
xrobot instance add QDU-Robomaster/CaptureFileCamera
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `ramfs` 填为 BSP 中用 `XR_REGISTER` 注册的 RamFS 对象名。帧布局用 constexpr 定义，必须与
回放数据的图像尺寸一致；默认标定和几何对应 720x540，因此这里使用 720x540 布局：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 720, .height = 540, .step = 2160, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/CaptureFileCamera
    id: capturefilecamera_0
    template_args:
      - ProjectConstexpr::FrameLayout
    args:
      - ramfs: ramfs
      - calibration: CaptureFileCamera<ProjectConstexpr::FrameLayout>::DefaultCalibration()
      - runtime: CaptureFileCamera<ProjectConstexpr::FrameLayout>::DefaultRuntime()
```

默认路径是相对进程工作目录的 `capture_frames.bin`、`capture_frames.csv` 和
`capture_imu.csv`。要指定文件，把 `runtime` 写成 YAML map。`RuntimeParam` 带构造函数，
因此 map 的键必须按顺序写出某个构造函数的参数名（带 `_in` 后缀）；字符串写成 C++ 字符串
字面量，`geometry_in` 可以引用另行定义的 `CameraTypes::FrameGeometry` 类型 constexpr，例如：

```yaml
runtime:
  file_path_in: '"./data/record_frames.bin"'
  frame_csv_path_in: '"./data/record_frames.csv"'
  imu_csv_path_in: '"./data/record_imu.csv"'
  camera_name_in: '"camera"'
  image_topic_name_in: '"camera_image"'
  imu_topic_name_in: '"camera_imu"'
  realtime_in: true
  loop_in: false
  max_frames_in: 0
  trigger_period_us_in: 10000
  geometry_in: 'ProjectConstexpr::FrameGeometry'
  replay_speed_in: 1.0
```

BSP 侧：

```cpp
XR_REGISTER(ramfs, LibXR::RamFS);
```

后续的 CameraFrameSync 实例用 `camera: capturefilecamera_0` 引用本实例，必须列在本实例之后。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/CaptureFileCamera`
（在 BSP 中）打印当前的构造函数。

## 测试

在打开 `BUILD_TESTING` 的 BSP 构建中，本模块加入 `capture_file_camera_replay_pacing_test`
（图像槽等待与停止）和 `capture_file_camera_runtime_param_compat_test`（`RuntimeParam`
两种构造布局），用 `ctest` 运行。

## 典型用途

- 用统一 raw frame-bin 内录包复现实机输入。
- 调试 `CameraFrameSync` 的图像和 IMU 对齐。
- 在 CI 或本地回放固定数据，检查检测、跟踪、预览等模块是否还能稳定运行。
