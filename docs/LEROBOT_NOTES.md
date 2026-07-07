# LeRobot 数据采集:工程优点与可复用模式

翻遍 lerobot v3.0 采集/存储代码(`datasets/`、`scripts/lerobot_record.py`)后的笔记 —— 它为什么健壮,
以及哪些模式值得搬进自己的采集工具。文末是一课反面教训。

## 为什么健壮(核心设计)

### 1. 采集与编码/落盘完全解耦 —— "不掉帧"的根本
控制循环只做三件轻活:读传感器 → 发指令 → 把帧丢进队列。**重活(图像编码、写盘、视频编码)在独立
进程/线程**:
- `num_image_writer_processes` / `num_image_writer_threads_per_camera` —— 图像异步落盘
- `VideoEncodingManager` + `streaming_encoding` + `ProcessPoolExecutor`(`_encode_video_worker`)—— 视频编码离线程
- 33ms 实时预算里绝不阻塞在 IO 上

> 启示:实时循环里任何 `encode/write/imwrite` 都应该丢给后台。我们的 `cameras_nonblocking` 优化就是
> 补上被 `async_read` 阻塞破坏的这一环(29.9→76.9Hz)。

### 2. Episode 缓冲 + 原子 保留/丢弃
`add_frame` 只进内存缓冲;`save_episode()` 才提交;`clear_episode_buffer()` 丢弃。
**坏 episode 永远进不了数据集**,重录零成本。

> 启示:采集的最小提交单元是"一条 episode",不是"一帧"。有了这个原语,闸门式(录完选保留/丢弃)、
> 出错自动丢弃重录,都只是几行控制流。

### 3. 视频优先存储 + 每模态独立编码
- 彩色:h264/hevc 有损(小)
- 深度:**12-bit 无损 hevc**(`gray12le` + `x265 lossless=1`)+ 量化元数据
  (`depth_min/max/shift/use_log`)。存的是量化码,读时按元数据 `dequantize` 回毫米。
- 省几十倍磁盘,还完整可复现(实测深度 round-trip 误差 = 纯量化 ~1mm)。

### 4. v3.0 分块列存(scalable)
不再"一 episode 一文件",而是 `data/chunk-{k}/file-{n}.parquet` + `videos/{key}/chunk-{k}/file-{n}.mp4`。
百万帧级也能高效随机访问、原生喂 HuggingFace `datasets` 训练。多条 episode 追加时用 `concatenate_video_files`
做**无重编码 remux**(stream copy)拼进同一个 chunk。

### 5. 时间戳 + 容差解码
每帧存 `timestamp`;解码按时间戳 seek + `tolerance_s` 容差,对抖动/丢帧鲁棒;后端可回退
(torchcodec→pyav)。→ 训练读取不会因个别帧时序偏差炸掉。

### 6. 声明式配置 + 注册表插件化
dataclass + draccus + `register_subclass`。新硬件(robot/teleop/camera)**注册即用,不用 fork**;
数据集 schema 从 processor pipeline **自动聚合**(`aggregate_pipeline_dataset_features`),不用手维护特征表。
→ 我们能把 `rebot_follower` / `orbbec` / `starai_to_rebot_leader` 插进去,全靠这个。

### 7. 异常安全的收尾
`@safe_stop_image_writer` 装饰器、`VideoEncodingManager` 上下文管理器、优雅 disconnect(回零)——
即使中途抛异常,编码器/写盘也会 flush,不留半截文件。

## 一课反面:健壮 ≠ 对依赖版本鲁棒

我们在这台机器上连撞 4 个 pyav 兼容坑(全在带深度存储路径):

| 报错 | 根因 | 修法 |
|---|---|---|
| `Conversion ... gray12le not supported` | 老 pyav 无 gray12le 的 `from_ndarray`/`to_ndarray` | 编码用 `VideoFrame` 构造器 + `write_u16_plane`;解码手动读 u16 plane |
| `Codec has no canonical_name` | 老 pyav Codec 只有 `.name` | `getattr(codec,"canonical_name",None) or codec.name` |
| `no add_stream_from_template` | 老 pyav 用 `add_stream(template=)` | `hasattr` 探测后回退 |

lerobot 逻辑健壮,但**用了较新 pyav API 却没做能力探测** → 对依赖版本硬耦合。
所有修补见 `lerobot_plugins/install_depthfix.sh`(能力探测 + 回退,幂等)。

> **通用启示:跨 FFI / 第三方库(尤其 pyav/ffmpeg 这种版本碎片化的)的调用,用 `hasattr` 能力探测 +
> 回退,别假设版本。** 我们自己的插件也应遵循(`getattr(config, "x", default)` 已在用)。

## 硬件韧性(这台机器踩出来的)

- **PCAN-USB `-71` USB 抖动**:适配器在共享 USB2 hub 上会 `Rx urb aborted (-71)` → `can0 removed` 重枚举
  → CAN 掉线。软件 USBDEVFS 复位救不了,需**物理重插**;根治是挪到独立 USB 口。
- **Orbbec 会话泄漏**:进程异常退出(core dump / `kill -9`)没走 `pipeline.stop()` → 下次开流拿不到帧。
  已在 `camera_orbbec.connect()` 做**自愈**(warmup 卡住自动 USB 复位重试,`reset_on_stall`)。
- **别用 `kill -9` 停采集**:用 `Ctrl-C`/`q` 优雅退出,相机/CAN 干净关闭,不泄漏、不 core dump。
