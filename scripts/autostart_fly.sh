#!/usr/bin/env bash
# D 题立体货架盘点 — 开机自启脚本
#
# 由 systemd / rc.local / 桌面自启项在开机后调用。做三件事：
#   1) 等系统起来（网络/串口/相机枚举完成）
#   2) source ROS humble + 本工作空间 install
#   3) 用 run_test.sh 的同款套路启动盘点 launch：
#        - 控制台输出 tee 到带时间戳日志（状态机叙事）
#        - 关键话题录 rosbag（事后回放/画曲线）
#        - 收到停止信号时让 bag 正常写完 metadata
#
# 默认起「要求1 整套基础部分」inventory_full（飞控底座 carto+uart+位置PID + 视觉 + 任务，
# A,B,C,D 四面遍历 + 落黑圆）。⚠ 这会上电即起飞控并进入起飞→遍历流程，现场务必确认安全/可随时接管。
# 想换 launch / 包（例：只起 QR→激光链路调试，不飞）：
#   AUTOSTART_PKG=inventory_control_pkg AUTOSTART_LAUNCH=qr_laser_test.launch.py ./autostart_fly.sh
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

LAUNCH_PKG="${AUTOSTART_PKG:-my_launch}"
LAUNCH_FILE="${AUTOSTART_LAUNCH:-inventory_full.launch.py}"

# ---- 要录的话题（D 题盘点排查用，见开发笔记）----
TOPICS=(
  /qr_vision/id            # 识别到的二维码编号 "1".."24"（=摄像头识别到没）
  /qr_vision/offset_norm   # 归一化像素偏移 x=ex(右正)/y=ey(下正)
  /qr_vision/aligned       # 是否已对准中心
  /qr_vision/laser_fired   # 激光打满 0.5s 后回报刚打的码（=激光有没有打，§10）
  /qr_vision/strict_vertical # 换行升/降时的纵向严判开关状态（§10）
  /qr_vision/enable        # 识别+激光总开关（mission 到位才开）
  /inventory_result        # 逐货位上报 编号=N,货位=XY（建表/LCD）
  /inventory_led           # 每盘一个 LED 亮灭
  /inventory_target        # 定向盘点抽取码编号/确认
  /inventory_target_slot   # 地面站下发货位 "C5"
  /inventory_status        # 飞机状态/心跳文本（待命/识别/盘点到哪步，地面站顶栏显示）
  /inventory_mode          # 地面站下发本轮任务模式 traverse/directed（飞机重启后据此起飞）
  /standoff/distance       # 雷达测板面距离 cm（standoff 闭环）
  /height                  # STM32 上报离地高度 cm（z 反馈）
  /target_position         # mission 下发目标点 [x,y,z,yaw]
  /mission_step            # STM32 回传任务步
  /target_velocity         # PID→飞控 目标速度(0x31)，复核 yaw180 机体系旋转(§9.3)
  /velocity_map            # 当前速度反馈(0x32)
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

  # 等外设枚举（串口 / 盘点用下视相机 /dev/video0）。不够再加大。
  sleep 8
  echo "[autostart] after sleep at $(date)"

  cd "$WS_ROOT" || { echo "[autostart] 找不到工作空间 $WS_ROOT"; exit 1; }

  # ROS humble + 本工作空间。.bashrc 在交互终端已 source，但开机自启是非交互
  # shell，这里必须显式 source。
  source /opt/ros/humble/setup.bash
  source "$WS_ROOT/install/setup.bash"

  # 跨机 DDS 环境（与地面站同 ROS_DOMAIN_ID）。只导出环境变量，不再 source ROS。
  source "$SCRIPT_DIR/setup_dds.sh"

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
