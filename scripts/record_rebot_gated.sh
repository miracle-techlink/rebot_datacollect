#!/usr/bin/env bash
# 闸门式采集单臂 reBot 双视角带深度数据集(每条 15s → 保留/丢弃 → 回车下一条)。
# 记录内容与 record_rebot.sh 相同(observation.state / images.wrist(+_depth) / images.front / action)。
#
# 用法:  PY=/path/to/env/python REPO_ID=用户名/数据集 TASK="任务描述" \
#           bash scripts/record_rebot_gated.sh [额外 --key=val ...]
#   环境变量: PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / REPO_ID / TASK / EPISODES /
#             EP_TIME(每条秒数,默认15) / PUSH / NO_DEPTH / WARMUP / FPS
#   性能优化开关(见 README「优化」):
#     CAM_FORMAT=mjpg|rgb|yuyv  腕部彩色在线格式(USB3 用 rgb 免 CPU 解码;USB2 必须 mjpg)。默认 mjpg
#     ALIGN_MODE=sw|hw          深度 D2C 对齐:hw=硬件(卸 CPU,需设备支持)。默认 sw
#     NO_DISPLAY=1              关 rerun(批量录制省主循环开销)
#     STREAM_ENCODE=1          流式视频编码(搬离主循环,多核并行;+ ENC_THREADS,默认2)
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
FPS="${FPS:-30}"
PUSH="${PUSH:-false}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: ${FPS}, width: 640, height: 480, color_format: ${CAM_FORMAT:-mjpg}, use_depth: ${USE_DEPTH}, align_mode: ${ALIGN_MODE:-sw}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: ${FPS}, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"

# 性能开关 → CLI
OPT_ARG=()
[ "${NO_DISPLAY:-0}" = "1" ] && DISPLAY_FLAG="false" || DISPLAY_FLAG="true"
if [ "${STREAM_ENCODE:-0}" = "1" ]; then
  OPT_ARG+=(--dataset.streaming_encoding=true --dataset.encoder_threads="${ENC_THREADS:-2}")
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$PY" "$SCRIPT_DIR/record_rebot_gated.py" \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  --robot.cameras="${CAMS}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --dataset.fps="${FPS}" --display_data="${DISPLAY_FLAG}" \
  --dataset.rgb_encoder.vcodec=h264 \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.single_task="${TASK}" \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.episode_time_s="${EP_TIME}" \
  --dataset.push_to_hub="${PUSH}" "${OPT_ARG[@]}" "$@"
