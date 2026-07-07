# rebot_datacollect

单臂 **Seeed reBot B601-RS** 的 [LeRobot](https://github.com/huggingface/lerobot) 遥操作数据采集套件:StarAI Violin 主臂 → reBot 从臂,腕部 Orbbec 深度 + 第二视角 USB 相机,录成标准 LeRobot 数据集(带对齐深度),用于 VLA / 模仿学习训练。

从 [`Galaxea_rebot_starai_tele`](https://github.com/miracle-techlink/Galaxea_rebot_starai_tele) 中把「单臂 reBot 采集」这条链路抽出来做成的独立、聚焦仓库。

## 特性

- **原生 LeRobot 接口**:`--robot.type=rebot_follower` + `--teleop.type=starai_to_rebot_leader`,直接配合官方 `lerobot-teleoperate` / `lerobot-record`,`--display_data` 免费给 rerun 可视化。
- **绝对映射遥操作**:leader 标定零位恒定对应从臂 home,进入遥操作即对上主臂**绝对位姿**(不是把「进入那一刻」当零位);启动这一跳由 teleop 端限速 ramp 平滑滑过去,不暴力弹射。见 `starai_to_rebot_leader`。
- **带深度**:腕部 Orbbec Gemini 305 输出彩色 + 软件 D2C 对齐深度(`observation.images.wrist` / `wrist_depth`,uint16 毫米),LeRobot 自动用深度编码器保存。
- **闸门式采集**:`record_rebot_gated.py` —— 每条固定时长 → 当场选保留/丢弃 → 回车开始下一条(比官方自动连录更适合精细任务)。

## 硬件

| 部件 | 说明 |
|---|---|
| 从臂 | Seeed reBot B601-RS(**RobStride** 电机,7 轴含夹爪),SocketCAN @ 1 Mbps,PCAN-USB(peak_usb) |
| 主臂 | StarAI Violin(飞特 UART 舵机,6 臂 + 1 夹爪),`/dev/ttyCH341USB0` |
| 腕部相机 | Orbbec Gemini 305(彩色 + 深度),**建议 USB3 口** |
| 第二视角 | 任意 UVC USB 相机(V4L2) |
| 主机 | 已在 Jetson Thor(Seeed reComputer J601)验证 |

## 依赖

```bash
# 在你的 lerobot conda/venv 里
pip install lerobot-robot-seeed-b601        # 官方 reBot RobStride follower(被 rebot_follower 继承)
pip install pyorbbecsdk2                      # Orbbec 相机(录深度需要)
pip install fashionstar-uart-servo            # StarAI Violin 主臂舵机(uservo)
# reBot CAN 需要 peak_usb 内核模块(PCAN-USB);Jetson 无此模块需自行 out-of-tree 编译
```

## 安装

```bash
# 1) 把插件装进 lerobot 源码树并注册
LEROBOT_SRC=/path/to/lerobot bash lerobot_plugins/install.sh
# 2) 装 Orbbec 相机插件(录深度用)
LEROBOT_SRC=/path/to/lerobot bash lerobot_plugins/install_orbbec.sh
```
> 注:插件是「拷进 lerobot 源码树」的方式,lerobot 升级会覆盖,升级后重跑 install。

## 快速开始

```bash
# 0) (Jetson)锁频提性能 —— 消除 DVFS 抖动,采集循环更容易稳到 30Hz
sudo bash scripts/maxn_lock.sh

# 1) 拉起 reBot CAN(自动找 PCAN 接口,USB 重枚举后 can 号会漂)
sudo bash scripts/setup_rebot_can.sh

# 2) 标定从臂(首次;类型必须是 rebot_follower)
#    lerobot-calibrate --robot.type=rebot_follower --robot.id=follower1 --robot.port=<canX> --robot.can_adapter=socketcan

# 3) 空跑遥操作,rerun 里确认视角 / 手感 / 夹爪方向
PY=/path/to/env/python CAN=<canX> FRONT_CAM=/dev/videoN bash scripts/teleop_rebot.sh

# 4) 闸门式采集(每条 15s → 保留/丢弃 → 回车下一条)
PY=/path/to/env/python CAN=<canX> FRONT_CAM=/dev/videoN \
  REPO_ID="你的用户名/数据集名" TASK="pick up the black object and place it in the box" \
  EPISODES=50 EP_TIME=15 PUSH=false \
  bash scripts/record_rebot_gated.sh
```

### 采集交互(在启动终端里按键)
- `▶ 回车开始录制` —— 摆好主臂/物体,回车开录
- 录制中 `→` 或 `Esc` = 提前结束本条(不想等满时长)
- `■ 回车/k=保留   d=丢弃重录   q=保存已录并退出`

数据集存 `~/.cache/huggingface/lerobot/<REPO_ID>_<时间戳>/`(LeRobot 每次采集自动加时间戳保证唯一)。

## 脚本一览

| 脚本 | 作用 |
|---|---|
| `scripts/teleop_rebot.sh` | 空跑遥操作 + rerun 双视角(不写数据集) |
| `scripts/record_rebot_gated.sh` / `.py` | **闸门式**采集(15s / 保留丢弃 / 回车下一条) |
| `scripts/record_rebot.sh` | 官方自动连录(方向键控制,连续 N 条) |
| `scripts/setup_rebot_can.sh` | 拉起 PCAN CAN 总线(自动找接口) |
| `scripts/maxn_lock.sh` | Jetson MAXN + 锁频(Tier-0 性能) |
| `scripts/usbreset_orbbec.py` | Orbbec 卡死时 USB 复位(免拔插) |

## 数据集内容

- `observation.state` — reBot 7 关节角(`shoulder_pan.pos` … `gripper.pos`)
- `observation.images.wrist` — 腕部 Orbbec 彩色 `(H,W,3)`
- `observation.images.wrist_depth` — 腕部对齐深度 `(H,W,1)` uint16 毫米
- `observation.images.front` — 第二视角彩色
- `action` — reBot 关节空间目标(与 `observation.state` 同帧;映射在 teleop 里完成 → 训练数据 action 与 obs 同坐标系)

## 常见问题

- **Orbbec 抓帧超时 / 卡死**:多为反复启停后管线卡死。`python scripts/usbreset_orbbec.py` 复位后重试;根治是插 **USB3 口**。
- **`can*` / `/dev/video*` 号变了**:USB 重新枚举会漂。`setup_rebot_can.sh` 自动找 PCAN 接口;相机用 `motorbridge-cli` / `v4l2-ctl --list-devices` 重认。
- **`Unsupported video codec: libsvtav1`**:某些 pyav 构建没有 svtav1。脚本已默认 `--dataset.rgb_encoder.vcodec=h264`(深度用 hevc)。
- **record loop < 30Hz**:先 `maxn_lock.sh`;把 Orbbec 挪 USB3、CAN/主臂串口与相机分开 USB 控制器;批量录制可 `--display_data=false`。

## 许可

插件基于 Apache-2.0(沿用 LeRobot / HuggingFace 头)。
