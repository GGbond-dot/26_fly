#!/usr/bin/env bash
# G 题 空地协同智能消防系统（无人机侧）— 开机自启脚本
#
# 由 systemd / rc.local / 桌面自启项在开机后调用。做三件事：
#   1) 等系统起来（网络/串口/相机枚举完成）
#   2) source ROS humble + 本工作空间 install
#   3) 用 run_test.sh 的同款套路启动消防 launch：
#        - 控制台输出 tee 到带时间戳日志（状态机叙事）
#        - 关键话题录 rosbag（事后回放/画曲线）
#        - 收到停止信号时让 bag 正常写完 metadata
#
# 默认起 my_launch/fire_full（飞控底座 carto+uart+位置PID + 消防任务 + 机车 UDP 桥 + 下视识别）。
# ✅ 上电后**不会自己起飞**：fire_full 默认 auto_start=false，飞机停在 WAIT_START，
#    等消防车按键（UDP 8893 → /fire_start）才起飞 → 18dm 蛇形全覆盖 → 返航降落。
# 组网：本机自建热点，机 = 10.42.0.1（网关），消防车 10.42.0.163 接进来。
#
# 想换 launch / 参数（自启也能传 launch 参数）：
#   AUTOSTART_LAUNCH_ARGS="enable_fire_task:=true" ./autostart_fly.sh   # 带视觉跑发挥部分
#   AUTOSTART_LAUNCH_ARGS="auto_start:=true" ./autostart_fly.sh         # 调试：不等按键直接飞
#   AUTOSTART_PKG=fire_link_pkg AUTOSTART_LAUNCH=fire_link.launch.py ./autostart_fly.sh  # 只调链路，不飞
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
LAUNCH_FILE="${AUTOSTART_LAUNCH:-fire_full.launch.py}"
# 额外的 launch 参数（空格分隔的 key:=value），例：AUTOSTART_LAUNCH_ARGS="enable_fire_task:=true"
read -r -a LAUNCH_ARGS <<< "${AUTOSTART_LAUNCH_ARGS:-}"

# ---- 要录的话题（G 题消防排查用）----
TOPICS=(
  /fire_status             # 飞机阶段/心跳文本（≥2Hz，断流即判离线；车端屏显也解析它）
  /fire_start              # 消防车按键启动（UDP 8893 → 本地话题）=按键到底有没有进来
  /car_status              # 车端任务状态字符串 ready/enroute/extinguishing/…（UDP 8890）
  /drone_pose              # [x_dm,y_dm] @1Hz 上报车端显示（基本要求3）
  /patrol_distance         # [dist_dm] @1Hz 累计巡逻里程（基本要求4）
  /fire_report             # 火源坐标 [x_dm,y_dm]（发挥2，只发一次×5）
  /fire/detected           # 视觉：本帧看到红色火花图案
  /fire/position_map       # 视觉反投影坐标 [x_cm,y_cm]（仅用于触发火情流程，不再上报车端）
  /fire/servo_error        # 闭环对中误差 [ex,ey,bx_cm,by_cm]=对中收敛过程，符号错也看这条
  /drop_package            # 抛灭火包标志位（飞控舵机帧 0x11）=舵机到底触发没
  /buzzer_led_control      # 机上报警 LED（发挥1 示警）
  /magnet/cmd              # 机腹激光笔（香橙派 GPIO pin13，1=亮/2=灭）
  /height                  # STM32 上报离地高度 cm（z 反馈）
  /target_position         # mission 下发目标点 [x,y,z,yaw]（map 系 cm）
  /mission_step            # STM32 回传任务步
  /target_velocity         # PID→飞控 目标速度(0x31)
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

  # 等外设枚举（串口 / 下视火源相机 /dev/video0 + 热点起来拿到 10.42.0.1）。不够再加大。
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

  echo "[autostart] 启动: ros2 launch $LAUNCH_PKG $LAUNCH_FILE ${LAUNCH_ARGS[*]}"
  echo "[autostart] 控制台日志 -> $CONSOLE_LOG"
  echo "[autostart] 数据包     -> $BAG_DIR  (回放: ros2 bag play '$BAG_DIR')"
  echo "[autostart] 录制话题:  ${TOPICS[*]}"
  echo "[autostart] === 自启动开始 ==="

  # 先起 bag（后台）。开 zstd 文件级压缩：其它话题也省空间，且不录整段 /scan。
  ros2 bag record --compression-mode file --compression-format zstd \
    -o "$BAG_DIR" "${TOPICS[@]}" &
  BAG_PID=$!

  # 起 launch（前台），输出同时进控制台日志和外层自启日志
  stdbuf -oL -eL ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" 2>&1 \
    | tee "$CONSOLE_LOG"
} 2>&1 | tee -a "$AUTO_LOG"
