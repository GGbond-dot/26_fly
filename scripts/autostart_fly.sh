#!/usr/bin/env bash
# G 题植保飞行器 — 开机自启脚本
#
# 由 systemd / rc.local / 桌面自启项在开机后调用。做三件事：
#   1) 等系统起来（网络/串口/相机枚举完成）
#   2) source ROS humble + 本工作空间 install
#   3) 用 run_test.sh 的同款套路启动 spray_basic.launch.py：
#        - 控制台输出 tee 到带时间戳日志（状态机叙事）
#        - 关键话题录 rosbag（事后回放/画曲线）
#        - 收到停止信号时让 bag 正常写完 metadata
#
# 想换 launch（如只联调 LED）：AUTOSTART_LAUNCH=led_digit_test.launch.py ./autostart_fly.sh
#
# 产物（默认 ~/fly_logs/）：
#   autostart_<时间戳>.log   开机自启外层日志（含 source/环境信息）
#   run_<时间戳>.log         launch 控制台日志
#   bag_<时间戳>/            rosbag 数据包

# 注意：不要开 set -u（nounset）。ROS 的 setup.bash 会引用未定义的
# AMENT_TRACE_SETUP_FILES，开了 -u 会在 source 这步直接报错退出。
set -o pipefail

# ---- 路径解析：脚本在 <ws>/scripts/ 下，ws 根 = 上一级（不写死 kian_ws）----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOG_DIR="${FLY_LOG_DIR:-$HOME/fly_logs}"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
AUTO_LOG="$LOG_DIR/autostart_${TS}.log"

LAUNCH_PKG="my_launch"
LAUNCH_FILE="${AUTOSTART_LAUNCH:-spray_basic.launch.py}"

# ---- 要录的话题（G 题 spray 数值排查用，见开发笔记 §5）----
TOPICS=(
  /height                  # STM32 上报离地高度 cm（z 反馈）
  /target_position         # mission 下发目标点 [x,y,z,yaw]
  /target_velocity         # 位置 PID 输出速度
  /active_controller       # 控制器接管状态
  /electromagnet_control   # 激光开关（复用电磁铁链路 0x33）
  /led_digit               # 条码数字 LED 闪烁帧
  /spray_allowed           # 下视见绿门控
  /barcode_text            # Code128 识别结果
  /pillar_detect_enable    # 柱子检测窗开/关（起飞后空中识别）
  /detected_pillars        # tf 版多杆 xy（按票数降序，取首个=最佳）
  /pillar_debug_points     # 检测窗内落进 bbox 的 map 系原始点（取代整段 /scan，离线重调聚类参数用）
  /mission_step            # STM32 回传任务步
)

# ---- 收尾：让 bag 正常落 metadata ----
BAG_PID=""
_CLEANED=0
cleanup() {
  [ "$_CLEANED" = "1" ] && return
  _CLEANED=1
  echo
  echo "[autostart] 收到停止信号，正在收尾..."
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null
    wait "$BAG_PID" 2>/dev/null
  fi
}
trap cleanup INT TERM EXIT

{
  echo "[autostart] started at $(date)"
  echo "[autostart] user=$(whoami)"
  echo "[autostart] ws=$WS_ROOT"
  echo "[autostart] DISPLAY=${DISPLAY:-}"

  # 等外设枚举（串口 / 下视相机 /dev/video2）。不够再加大。
  sleep 8
  echo "[autostart] after sleep at $(date)"

  cd "$WS_ROOT" || { echo "[autostart] 找不到工作空间 $WS_ROOT"; exit 1; }

  # ROS humble + 本工作空间。.bashrc 在交互终端已 source，但开机自启是非交互
  # shell，这里必须显式 source。
  source /opt/ros/humble/setup.bash
  source "$WS_ROOT/install/setup.bash"

  # 视觉节点要弹预览窗 → 给 X 环境兜底
  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

  # 日志立即刷盘 + 带时间戳，断电/Ctrl-C 不丢尾巴
  export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
  export RCUTILS_LOGGING_BUFFERED_STREAM=0

  CONSOLE_LOG="$LOG_DIR/run_${TS}.log"
  BAG_DIR="$LOG_DIR/bag_${TS}"

  echo "[autostart] 启动: ros2 launch $LAUNCH_PKG $LAUNCH_FILE"
  echo "[autostart] 控制台日志 -> $CONSOLE_LOG"
  echo "[autostart] 数据包     -> $BAG_DIR  (回放: ros2 bag play '$BAG_DIR')"
  echo "[autostart] 录制话题:  ${TOPICS[*]}"
  echo "[autostart] === 自启动开始 ==="

  # 先起 bag（后台）。开 zstd 文件级压缩：其它话题也省空间，且不录整段 /scan。
  ros2 bag record --compression-mode file --compression-format zstd \
    -o "$BAG_DIR" "${TOPICS[@]}" &
  BAG_PID=$!

  # 起 launch（前台），输出同时进控制台日志和外层自启日志
  stdbuf -oL -eL ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" 2>&1 \
    | tee "$CONSOLE_LOG"
} 2>&1 | tee -a "$AUTO_LOG"
