#!/usr/bin/env bash
# 一键试飞 + 全量留日志脚本
#
# 同时做两件事，Ctrl-C 一起收尾：
#   1) ros2 launch ...        控制台输出 tee 到带时间戳的日志文件（状态机叙事）
#   2) ros2 bag record ...    把关键话题录成 bag（原始数据，事后回放/画曲线）
#
# 用法：
#   ./scripts/run_test.sh                      # 默认跑 demo3.launch.py
#   ./scripts/run_test.sh demo_basic.launch.py # 跑别的 launch
#   ./scripts/run_test.sh demo3.launch.py enable_debug_topic_info:=false   # 透传 launch 参数
#
# 产物（默认在 ~/fly_logs/）：
#   run_<时间戳>.log     控制台日志
#   bag_<时间戳>/        rosbag 数据包

set -uo pipefail

# ---- 路径解析：脚本在 <ws>/scripts/ 下，ws 根 = 上一级 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LAUNCH_PKG="my_launch"
LAUNCH_FILE="${1:-demo3.launch.py}"
shift || true              # 余下参数透传给 ros2 launch
LAUNCH_ARGS=("$@")

# 环境变量（ROS humble + 本工作空间）已在 .bashrc 里 source 好，这里不重复。

# 日志带时间戳 + 行缓冲立即刷盘（中途断电/Ctrl-C 也不丢尾巴）
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# ---- 产物路径 ----
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${FLY_LOG_DIR:-$HOME/fly_logs}"
mkdir -p "$LOG_DIR"
CONSOLE_LOG="$LOG_DIR/run_${TS}.log"
BAG_DIR="$LOG_DIR/bag_${TS}"

# ---- 要录的话题（数值排查用）----
TOPICS=(
  /height                       # 点阵激光高度（测柱高输入）
  /laser_array/ground_height    # 面阵地面高度（z 反馈）
  /circle_area_ratio            # 铁片占比（识别/排序）
  /fine_data                    # 视觉像素偏差（对中心）
  /detected_pillars             # 检测到的柱子 xy
  /target_position              # mission 下发目标点
  /target_velocity              # PID 输出速度
  /mission_step                 # STM32 回传的任务步
)

# ---- 收尾 ----
BAG_PID=""
_CLEANED=0
cleanup() {
  [ "$_CLEANED" = "1" ] && return
  _CLEANED=1
  echo
  echo "[run] 收到停止信号，正在收尾..."
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null   # SIGINT 让 bag 正常写完 metadata
    wait "$BAG_PID" 2>/dev/null
  fi
  echo "[run] 控制台日志: $CONSOLE_LOG"
  echo "[run] 数据包:     $BAG_DIR  (回放: ros2 bag play '$BAG_DIR')"
}
trap cleanup INT TERM EXIT

echo "[run] 工作空间: $WS_ROOT"
echo "[run] 启动:     ros2 launch $LAUNCH_PKG $LAUNCH_FILE ${LAUNCH_ARGS[*]:-}"
echo "[run] 控制台日志 -> $CONSOLE_LOG"
echo "[run] 数据包     -> $BAG_DIR"
echo "[run] 录制话题:  ${TOPICS[*]}"
echo "[run] === 试飞开始，按 Ctrl-C 结束并收尾 ==="

# 先起 bag（后台）
ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" &
BAG_PID=$!

# 起 launch（前台），输出同时进控制台和日志文件
stdbuf -oL -eL ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" 2>&1 \
  | tee "$CONSOLE_LOG"
