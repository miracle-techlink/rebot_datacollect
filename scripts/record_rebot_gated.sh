#!/usr/bin/env bash
# 闸门式采集单臂 reBot 双视角带深度数据集(每条 15s → 保留/丢弃 → 回车下一条)。
# 记录内容与 record_rebot.sh 相同(observation.state / images.wrist(+_depth) / images.front / action)。
#
# 用法:  PY=/path/to/env/python REPO_ID=用户名/数据集 TASK="任务描述" \
#           bash scripts/record_rebot_gated.sh [额外 --key=val ...]
#   环境变量: PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / REPO_ID / TASK / EPISODES /
#             EP_TIME(每条秒数,默认15) / PUSH / NO_DEPTH / WARMUP
set -e
PY="${PY:-python}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can0}"                                   # PCAN reBot 总线(USB 重枚举后现在是 can0)
WRIST_CAM="${WRIST_CAM:-CV275610002L}"
FRONT_CAM="${FRONT_CAM:-/dev/video10}"               # 1080P USB 相机(USB 重枚举后现在是 /dev/video10)
REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"                             # 每条固定时长(秒)
PUSH="${PUSH:-false}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg, use_depth: ${USE_DEPTH}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: 30, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$PY" "$SCRIPT_DIR/record_rebot_gated.py" \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  --robot.cameras="${CAMS}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --dataset.fps=30 --display_data=true \
  --dataset.rgb_encoder.vcodec=h264 \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.single_task="${TASK}" \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.episode_time_s="${EP_TIME}" \
  --dataset.push_to_hub="${PUSH}" "$@"
