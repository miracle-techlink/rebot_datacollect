#!/usr/bin/env bash
# 官方 lerobot-teleoperate 驱动单臂 rebot_follower(StarAI 主臂 → reBot B601-RS),
# 双视角进 rerun:腕部 Orbbec(彩色+深度)+ 一路 USB 摄像头(第二视角,彩色)。
# Ctrl-C 停(reBot 按官方 follower 断开逻辑处理)。
#
# 用法:  PY=/path/to/lerobot-env/python  bash scripts/teleop_rebot.sh  [额外 --key=val ...]
#   环境变量(可选覆盖): PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / NO_CAM=1 / NO_DEPTH=1
#
# 前提: bash lerobot_plugins/install.sh + install_orbbec.sh;先用 seeed_b601_rs_follower 标定过;
#       reBot CAN 起来(sudo ip link set can5 up type can bitrate 1000000 restart-ms 100)。
set -e
PY="${PY:-python}"                                    # 指向装了 lerobot 的 conda env python(如 ~/miniconda3/envs/lerobot/bin/python)
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can5}"
WRIST_CAM="${WRIST_CAM:-CV275610002L}"                # reBot 腕部 Orbbec 深度相机序列号
FRONT_CAM="${FRONT_CAM:-/dev/video4}"                 # 第二视角 USB 摄像头(SN0002 1080P = /dev/video4)

# rerun viewer 需在 PATH 里(与 env python 同目录)
BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
CAM_ARG=()
if [ "${NO_CAM:-0}" != "1" ]; then
  # USB2 链路务必 color_format: mjpg;深度会对齐进彩色帧(640x480)。想 30Hz 深度请把 Orbbec 插到 USB3 口。
  # front 用 v4l2 后端(默认 ANY 后端会让 set(width) 返回 False 而报错)+ MJPG(省 USB2 带宽)
  CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg, use_depth: ${USE_DEPTH}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: 30, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"
  CAM_ARG=(--robot.cameras="${CAMS}")
fi

exec lerobot-teleoperate \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  "${CAM_ARG[@]}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --fps=30 --display_data=true "$@"
