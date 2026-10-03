# CaptureFileCamera

文件回放相机：回放内录的图像与 IMU 数据并发布 / File replay camera that publishes recorded images and IMU data

## 1. 模块作用 / Purpose

CaptureFileCamera 是 `CameraBase<FrameLayoutV>` 的派生类，从内录包读取图像和 IMU 数据，按录制时间戳回放，并通过 CameraBase 接口发布。内录包由 `frames.bin`、`frames.csv` 和 `imu.csv` 组成（格式见第 2 节）；`frame_csv_path` 为空时，`file_path` 作为视频文件打开，视频帧与 IMU CSV 的行按序号配对。

构造时，CaptureFileCamera 加载并校验 IMU CSV、帧索引 CSV 和帧数据 bin（或打开视频），随后启动一个后台线程回放。任何加载或校验错误都在构造时抛出异常。回放线程对每一帧先发布原始 IMU，再把 BGR8 图像写入 CameraBase 的图像槽并提交，因此图像与 IMU 复现实机相机的输入，可用于调试 `CameraFrameSync` 的同步和回归视觉流程。

`realtime` 为 `true` 时，每一轮回放以第一帧为计时起点，在发布每一帧之前等待到该帧的录制相对时间除以 `replay_speed`。循环播放时重新计时，录制时间戳仍回到文件起点。回放线程处理较慢时，帧按序逐帧提交。等待使用毫秒级时钟，无法表示的目标时间会记录错误并停止回放。`realtime` 为 `false` 时不限速。`replay_speed` 只改变播放节奏，图像和 IMU 的录制时间戳以及档位触发周期保持原值。

CameraBase 的图像池没有空槽时，回放线程保留当前已解码的帧，每 1 ms 重试一次。等待期间该帧的 IMU 不会重复发布，输入索引也保持不变；析构时的停止请求可以中断等待。

读取或解码失败、帧尺寸与布局不符时，回放停止。到达文件末尾时，`loop` 为 `false` 则停止，为 `true` 则从头开始；提交帧数达到 `max_frames`（非 0）时停止。

环境变量可以在测试中覆盖部分参数：

- `CAPTURE_FILE_CAMERA_MAX_FRAMES`：大于 0 时作为 `max_frames`。
- `CAPTURE_FILE_CAMERA_REALTIME`：取值 `0` 时关闭限速，其他取值开启限速。

文件相机只提供一个固定档位 `WIDE`，触发周期为 `trigger_period_us`。`SwitchProfile(WIDE)` 返回该档位的逐帧 geometry；请求其他档位返回 `NOT_SUPPORT`，回放状态不变。`SetExposure()` 与 `SetGain()` 为空实现，用于满足 CameraBase 接口。

`OnMonitor()` 打印累计提交帧数、已发布 IMU 组数、运行状态和本周期提交帧数，以及读帧、BGR 转换、IMU 发布、图像提交和限速等待的耗时统计（单位 us）。

CaptureFileCamera derives from `CameraBase<FrameLayoutV>`. It reads images and IMU data from a recording, replays them according to the recorded timestamps and publishes them through the CameraBase interface. A recording consists of `frames.bin`, `frames.csv` and `imu.csv` (format in section 2). When `frame_csv_path` is empty, `file_path` is opened as a video file and the video frames are paired with the rows of the IMU CSV by index.

At construction, CaptureFileCamera loads and validates the IMU CSV, the frame index CSV and the frame data bin (or opens the video), and then starts a background thread for the replay. Any loading or validation error is thrown as an exception at construction. For every frame the replay thread first publishes the raw IMU, then writes the BGR8 image into a CameraBase image slot and commits it. The images and the IMU data thus reproduce the input of a real camera, which supports debugging the synchronization of `CameraFrameSync` and regression runs of the vision pipeline.

With `realtime` set to `true`, each replay round takes the first frame as the timing origin and waits before publishing each frame until the recorded relative time of that frame divided by `replay_speed`. When looping, the timing restarts and the recorded timestamps return to the start of the file. When the replay thread runs late, frames are committed one after another in order. The wait uses a millisecond clock, and a target time that cannot be represented is logged as an error and stops the replay. With `realtime` set to `false` the replay is not rate limited. `replay_speed` only changes the replay pace; the recorded timestamps of images and IMU data and the trigger period of the profile keep their values.

When the CameraBase image pool has no free slot, the replay thread keeps the current decoded frame and retries every 1 ms. The IMU of that frame is not published again during the wait and the input index stays unchanged; a stop request from the destructor can interrupt the wait.

The replay stops when reading or decoding fails or a frame size does not match the layout. At the end of the file it stops when `loop` is `false` and restarts from the beginning when `loop` is `true`; it also stops when the number of committed frames reaches `max_frames` (non-zero).

Environment variables can override some parameters in tests:

- `CAPTURE_FILE_CAMERA_MAX_FRAMES`: used as `max_frames` when greater than 0.
- `CAPTURE_FILE_CAMERA_REALTIME`: the value `0` disables rate limiting, any other value enables it.

The file camera provides one fixed profile `WIDE`, whose trigger period is `trigger_period_us`. `SwitchProfile(WIDE)` returns the per-frame geometry of that profile; any other profile returns `NOT_SUPPORT` and the replay state is unchanged. `SetExposure()` and `SetGain()` are empty implementations that satisfy the CameraBase interface.

`OnMonitor()` prints the total number of committed frames, the number of published IMU groups, the running state and the number of frames committed in the period, followed by timing statistics (in us) for frame reading, BGR conversion, IMU publishing, image commit and rate-limit waiting.

## 2. 内录包格式 / Recording Format

统一内录包由三个文件组成：

- `file_path` 指向 `*_frames.bin`，保存图像数据。
- `frame_csv_path` 指向 `*_frames.csv`，保存帧索引。
- `imu_csv_path` 指向 `*_imu.csv`，保存 IMU 数据。

帧索引 CSV 的列顺序为：

```text
frame_index,camera_timestamp_us,offset_bytes,size_bytes[,codec]
```

`offset_bytes` 与 `size_bytes` 指向 bin 文件中的一帧图像，构造时检查其落在文件范围内。`codec` 为 `raw`（不区分大小写），或缺省且 `size_bytes` 等于 `CameraBase::image_bytes` 时，该帧按未压缩 BGR8 读取，大小必须等于 `image_bytes`。其他记录交给 OpenCV `imdecode` 解码，8 位灰度、BGR、BGRA 统一转为 BGR8，解码结果的尺寸必须与布局一致。

IMU CSV 的列顺序为：

```text
timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
```

- `timestamp_us`：传感器时间戳，单位 us。
- `qw,qx,qy,qz`：姿态四元数，顺序为 `wxyz`。
- `gx,gy,gz`：角速度，单位 rad/s。
- `ax,ay,az`：线加速度，单位 m/s^2。

CaptureFileCamera 用帧索引 CSV 的 `camera_timestamp_us` 与 IMU CSV 的 `timestamp_us` 对齐，回放包含能找到同 timestamp IMU 的图像帧，其余帧被跳过，启动日志给出跳过数量。没有任何对齐帧时构造失败。图像时间戳为 `camera_timestamp_us`。

视频模式（`frame_csv_path` 为空）下，`file_path` 由 OpenCV 按视频打开，第 N 帧与 IMU CSV 的第 N 行配对，图像时间戳取该 IMU 行的 `timestamp_us`。视频尺寸必须与布局一致，解码结果统一转为 BGR8。

两种 CSV 都可以包含空行、以 `#` 开头的注释行，以及第一条数据之前的一行表头。文件以无换行结束的最后一行无法解析时，该行被忽略并记录警告；其他无法解析的行使构造失败。

A recording consists of three files:

- `file_path` points to `*_frames.bin`, which stores the image data.
- `frame_csv_path` points to `*_frames.csv`, which stores the frame index.
- `imu_csv_path` points to `*_imu.csv`, which stores the IMU data.

The columns of the frame index CSV are:

```text
frame_index,camera_timestamp_us,offset_bytes,size_bytes[,codec]
```

`offset_bytes` and `size_bytes` locate one image in the bin file; the construction checks that they lie within the file. When `codec` is `raw` (case-insensitive), or is omitted and `size_bytes` equals `CameraBase::image_bytes`, the frame is read as uncompressed BGR8 and its size must equal `image_bytes`. All other records are decoded by OpenCV `imdecode`; 8-bit grayscale, BGR and BGRA are converted to BGR8, and the decoded size must match the layout.

The columns of the IMU CSV are:

```text
timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
```

- `timestamp_us`: sensor timestamp in us.
- `qw,qx,qy,qz`: attitude quaternion in `wxyz` order.
- `gx,gy,gz`: angular velocity in rad/s.
- `ax,ay,az`: linear acceleration in m/s^2.

CaptureFileCamera aligns the `camera_timestamp_us` of the frame index CSV with the `timestamp_us` of the IMU CSV. The replay contains the image frames for which an IMU with the same timestamp exists, the other frames are skipped, and the startup log reports the number of skipped frames. The construction fails when no frame is aligned. The image timestamp is `camera_timestamp_us`.

In video mode (`frame_csv_path` empty), `file_path` is opened as a video by OpenCV, frame N is paired with row N of the IMU CSV, and the image timestamp is the `timestamp_us` of that IMU row. The video size must match the layout, and the decoded frames are converted to BGR8.

Both CSV files may contain empty lines, comment lines starting with `#`, and one header line before the first data row. A last line of the file that ends without a line break and cannot be parsed is ignored with a warning; any other line that cannot be parsed makes the construction fail.

## 3. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CaptureFileCamera : public CameraBase<FrameLayoutV>;

explicit CaptureFileCamera(
    LibXR::RamFS& ramfs,
    CameraCalibration calibration = DefaultCalibration(),
    RuntimeParam runtime = DefaultRuntime());  // 节选 / excerpt
```

模板参数：

- `FrameLayoutV`：帧布局 `CameraTypes::FrameLayout`，为紧密排列的 BGR8（`step == width * 3`，编译期检查），与回放数据的图像尺寸一致。

依赖：

- `ramfs`：`LibXR::RamFS`，CameraBase 在其中注册相机命令文件。

配置参数：

- `calibration`：原生传感器坐标系下的 `CameraCalibration`，默认 `DefaultCalibration()`：1440x1080，`fx ≈ fy ≈ 2328.7`，`PLUMB_BOB` 畸变模型，五项畸变系数。
- `runtime`：`RuntimeParam`，字段见下表。

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `file_path` | `"capture_frames.bin"` | 帧数据 bin 路径；`frame_csv_path` 为空时为视频路径。 |
| `frame_csv_path` | `"capture_frames.csv"` | 帧索引 CSV 路径；为空时使用视频模式。 |
| `imu_csv_path` | `"capture_imu.csv"` | IMU CSV 路径。 |
| `camera_name` | `"camera"` | 相机名，也是原始 IMU Topic 的前缀和 RamFS 命令文件名。 |
| `image_topic_name` | `"camera_image"` | 图像 Topic 名称。 |
| `imu_topic_name` | `"camera_imu"` | 传给 CameraBase 的同步 IMU Topic 名称。 |
| `realtime` | `true` | 按录制时间戳限速回放。 |
| `loop` | `false` | 到达文件末尾后从头开始。 |
| `max_frames` | `0` | 最大提交帧数，`0` 表示不限制。 |
| `trigger_period_us` | `10000` | 单档对应的图像触发周期，单位 us，非零。 |
| `geometry` | 见下 | 帧坐标到原生传感器坐标的固定映射，复制到每一帧。 |
| `replay_speed` | `1.0` | 回放倍率，有限正数；`0.5` 为半速，`2.0` 为两倍速。 |

`geometry` 的默认值取布局的宽、高和步长，ROI 偏移为 0，横纵下采样均为 2，`flags`、`reserved` 和采样相位为 0。构造时校验 geometry 的尺寸、步长、下采样、ROI 与原生标定范围；默认值对应 1440x1080 原生标定下的 720x540 布局。

`RuntimeParam` 有三种构造方式：默认构造；按 `file_path_in, frame_csv_path_in, imu_csv_path_in, camera_name_in, image_topic_name_in, imu_topic_name_in, realtime_in, loop_in, max_frames_in, trigger_period_us_in, geometry_in, replay_speed_in` 顺序给出全部字段（`replay_speed_in` 默认 `1.0`）；以及省略 `trigger_period_us_in` 的同一顺序（`geometry_in` 紧跟 `max_frames_in`），此时触发周期取 `10000`。

Template parameter:

- `FrameLayoutV`: the frame layout `CameraTypes::FrameLayout`, tightly packed BGR8 (`step == width * 3`, checked at compile time), matching the image size of the replayed data.

Dependency:

- `ramfs`: `LibXR::RamFS` in which CameraBase registers the camera command file.

Configuration parameters:

- `calibration`: the `CameraCalibration` in the native sensor coordinate system, default `DefaultCalibration()`: 1440x1080, `fx ≈ fy ≈ 2328.7`, `PLUMB_BOB` distortion model, five distortion coefficients.
- `runtime`: `RuntimeParam`, with the fields in the table below.

| Field | Default | Meaning |
| --- | --- | --- |
| `file_path` | `"capture_frames.bin"` | Path of the frame data bin; the video path when `frame_csv_path` is empty. |
| `frame_csv_path` | `"capture_frames.csv"` | Path of the frame index CSV; an empty value selects video mode. |
| `imu_csv_path` | `"capture_imu.csv"` | Path of the IMU CSV. |
| `camera_name` | `"camera"` | Camera name, also the prefix of the raw IMU Topics and the name of the RamFS command file. |
| `image_topic_name` | `"camera_image"` | Name of the image Topic. |
| `imu_topic_name` | `"camera_imu"` | Name of the synchronized IMU Topic passed to CameraBase. |
| `realtime` | `true` | Rate limit the replay by the recorded timestamps. |
| `loop` | `false` | Restart from the beginning at the end of the file. |
| `max_frames` | `0` | Maximum number of committed frames; `0` means unlimited. |
| `trigger_period_us` | `10000` | Image trigger period of the single profile in us, non-zero. |
| `geometry` | see below | Fixed mapping from frame coordinates to native sensor coordinates, copied to every frame. |
| `replay_speed` | `1.0` | Replay speed factor, a finite positive number; `0.5` is half speed, `2.0` is double speed. |

The default `geometry` takes the width, height and step of the layout, uses ROI offsets of 0 and a decimation of 2 in both directions, and sets `flags`, `reserved` and the sample phases to 0. At construction the size, step, decimation and ROI of the geometry are validated against the native calibration range; the default corresponds to a 720x540 layout under the 1440x1080 native calibration.

`RuntimeParam` has three ways of construction: default construction; all fields in the order `file_path_in, frame_csv_path_in, imu_csv_path_in, camera_name_in, image_topic_name_in, imu_topic_name_in, realtime_in, loop_in, max_frames_in, trigger_period_us_in, geometry_in, replay_speed_in` (`replay_speed_in` defaults to `1.0`); and the same order without `trigger_period_us_in` (`geometry_in` directly follows `max_frames_in`), in which case the trigger period is `10000`.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.image_topic_name`（默认 `camera_image`） | 发布 | `const SharedFrame*`（CameraBase） | 每提交一帧图像发布一次，指针仅在同步回调期间有效 |
| `<camera_name>_gyro` | 发布 | `Eigen::Matrix<float, 3, 1>` | 原始角速度，单位 rad/s，Topic timestamp 为 CSV 的 `timestamp_us` |
| `<camera_name>_accl` | 发布 | `Eigen::Matrix<float, 3, 1>` | 原始线加速度，单位 m/s^2，Topic timestamp 为 CSV 的 `timestamp_us` |
| `<camera_name>_quat` | 发布 | `LibXR::Quaternion<float>` | 原始姿态四元数，Topic timestamp 为 CSV 的 `timestamp_us` |

原始 IMU Topic 位于默认 domain（`libxr_def_domain`）。CaptureFileCamera 发布原始 IMU。与 CameraFrameSync 配合时选择 `LATEST_IMU` 模式，并把 `host_topic_domain_name` 设为 `"libxr_def_domain"`。

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.image_topic_name` (default `camera_image`) | Publish | `const SharedFrame*` (CameraBase) | Published once per committed image; the pointer is valid only during the synchronous callback |
| `<camera_name>_gyro` | Publish | `Eigen::Matrix<float, 3, 1>` | Raw angular velocity in rad/s, Topic timestamp is the CSV `timestamp_us` |
| `<camera_name>_accl` | Publish | `Eigen::Matrix<float, 3, 1>` | Raw linear acceleration in m/s^2, Topic timestamp is the CSV `timestamp_us` |
| `<camera_name>_quat` | Publish | `LibXR::Quaternion<float>` | Raw attitude quaternion, Topic timestamp is the CSV `timestamp_us` |

The raw IMU Topics are in the default domain (`libxr_def_domain`). CaptureFileCamera publishes the raw IMU. Together with CameraFrameSync, the `LATEST_IMU` mode is selected and `host_topic_domain_name` is set to `"libxr_def_domain"`.

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/CaptureFileCamera` 与 `xrobot sync` 写出的实例以 `DefaultCalibration()` 和 `DefaultRuntime()` 作为 `calibration` 与 `runtime` 的默认值。以下实例取自 `bsp-linux-autoaim-replay` 的配置，使用视频模式（`frame_csv_path_in` 为空），`MainFrameLayout`、`MainCameraCalibration` 与 `MainFrameGeometry` 在该配置的 `constexprs` 中定义，`ramfs` 为 BSP 中用 `XR_REGISTER`（硬件注册）注册的 `LibXR::RamFS` 对象名：

```yaml
modules:
  - module: QDU-Robomaster/CaptureFileCamera
    id: camera
    template_args:
      - AutoAimRunConfig::MainFrameLayout
    args:
      - ramfs: ramfs
      - calibration: AutoAimRunConfig::MainCameraCalibration
      - runtime:
          file_path_in: "./data/camera_internal_recording_20260428/damo_clean.avi"
          frame_csv_path_in: ""
          imu_csv_path_in: "./data/camera_internal_recording_20260428/damo_imu.csv"
          camera_name_in: "capturefile_camera"
          image_topic_name_in: "capturefile_image"
          imu_topic_name_in: "capturefile_imu"
          realtime_in: true
          loop_in: false
          max_frames_in: 0
          geometry_in: AutoAimRunConfig::MainFrameGeometry
          replay_speed_in: 1.0
```

`runtime` 写成 YAML 映射时，键为所选构造函数的参数名（带 `_in` 后缀），顺序与该构造函数一致；字符串写成带引号的 C++ 字符串字面量。使用 `frames.bin` 内录包时，`file_path_in` 指向 `*_frames.bin`，`frame_csv_path_in` 指向 `*_frames.csv`。后续的 CameraFrameSync 实例以 `camera: camera` 引用本实例，须列在本实例之后。

The instance written by `xrobot instance add QDU-Robomaster/CaptureFileCamera` and `xrobot sync` uses `DefaultCalibration()` and `DefaultRuntime()` as the defaults of `calibration` and `runtime`. The following instance is taken from the configuration of `bsp-linux-autoaim-replay`. It uses video mode (`frame_csv_path_in` is empty); `MainFrameLayout`, `MainCameraCalibration` and `MainFrameGeometry` are defined in the `constexprs` of that configuration, and `ramfs` is the name of the `LibXR::RamFS` object registered in the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: QDU-Robomaster/CaptureFileCamera
    id: camera
    template_args:
      - AutoAimRunConfig::MainFrameLayout
    args:
      - ramfs: ramfs
      - calibration: AutoAimRunConfig::MainCameraCalibration
      - runtime:
          file_path_in: "./data/camera_internal_recording_20260428/damo_clean.avi"
          frame_csv_path_in: ""
          imu_csv_path_in: "./data/camera_internal_recording_20260428/damo_imu.csv"
          camera_name_in: "capturefile_camera"
          image_topic_name_in: "capturefile_image"
          imu_topic_name_in: "capturefile_imu"
          realtime_in: true
          loop_in: false
          max_frames_in: 0
          geometry_in: AutoAimRunConfig::MainFrameGeometry
          replay_speed_in: 1.0
```

When `runtime` is written as a YAML mapping, the keys are the parameter names of the chosen constructor (with the `_in` suffix) in the order of that constructor, and strings are written as quoted C++ string literals. With a `frames.bin` recording, `file_path_in` points to `*_frames.bin` and `frame_csv_path_in` points to `*_frames.csv`. A following CameraFrameSync instance references this instance with `camera: camera` and is listed after it.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CameraBase`：相机基类与共享图像槽。
- `xrobot-org/DurationStatistics`：`OnMonitor()` 中的耗时统计。
- OpenCV 4（`core`、`imgproc`、`imgcodecs`、`videoio`）与 Eigen。
- LibXR。

硬件：本地文件系统上的内录包；`ramfs` 由 BSP 注册。

Dependencies:

- `QDU-Robomaster/CameraBase`: camera base class and shared image slots.
- `xrobot-org/DurationStatistics`: timing statistics in `OnMonitor()`.
- OpenCV 4 (`core`, `imgproc`, `imgcodecs`, `videoio`) and Eigen.
- LibXR.

Hardware: a recording on the local file system; `ramfs` is registered by the BSP.
