#!/usr/bin/env bash
# D题 盘点无人机 — 地面站（diansai_terminal）开机自启脚本
#
# 与飞机端 autostart_fly.sh 对称：由香橙派「会话与启动」（XFCE Session and Startup
# 的自动启动应用）在登录后调用。做四件事：
#   1) 等系统/网络/X 桌面起来（DDS 要联网、PyQt5 GUI 要 X）
#   2) source ROS humble（地面站是纯 rclpy + PyQt5，不需要 colcon install）
#   3) source setup_dds.sh —— 与飞机端同 ROS_DOMAIN_ID，跨机才能互相发现
#   4) 全屏启动 ground_station.main，控制台输出 tee 到带时间戳日志
#
# 添加到自启动（XFCE）：设置 → 会话和启动 → 自动启动的应用程序 → 添加：
#     命令： /home/orangepi/<...>/diansai_terminal/scripts/autostart_ground_station.sh
#   （路径按地面站香橙派上仓库实际位置填；脚本自身会解析所在目录，不写死。）
#
# 多网口发现不稳时，给本脚本传 NET_IFACE 绑定网卡（与飞机端同款逃生口）：
#     NET_IFACE=wlan0 ./autostart_ground_station.sh
#
# 产物（默认 ~/ground_station_logs/）：
#   autostart_gs_<时间戳>.log   自启外层日志（含 source / 环境信息 + GUI 控制台）

# 注意：不要开 set -u（nounset）。ROS 的 setup.bash 引用未定义变量，开 -u 会在 source 报错退出。
set -o pipefail

# ---- 路径解析：脚本在 <terminal>/scripts/ 下，terminal 根 = 上一级（不写死路径）----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TERMINAL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOG_DIR="${GS_LOG_DIR:-$HOME/ground_station_logs}"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
AUTO_LOG="$LOG_DIR/autostart_gs_${TS}.log"

{
  echo "[autostart-gs] started at $(date)"
  echo "[autostart-gs] user=$(whoami)"
  echo "[autostart-gs] terminal=$TERMINAL_ROOT"
  echo "[autostart-gs] DISPLAY=${DISPLAY:-}"

  # 等网络 + X 桌面就绪。XFCE 自启有时早于网络连上，不够再加大。
  sleep 8
  echo "[autostart-gs] after sleep at $(date)"

  cd "$TERMINAL_ROOT" || { echo "[autostart-gs] 找不到地面站目录 $TERMINAL_ROOT"; exit 1; }

  # ROS humble。开机自启是非交互 shell，.bashrc 不一定 source，这里必须显式 source。
  source /opt/ros/humble/setup.bash

  # 跨机 DDS 环境（与飞机端 scripts/setup_dds.sh 同 ROS_DOMAIN_ID）。
  # 只导出环境变量，不再 source ROS。NET_IFACE 透传给它做网卡绑定。
  source "$SCRIPT_DIR/setup_dds.sh"

  # PyQt5 GUI 要 X 环境兜底（自启一般已在 X 会话内，DISPLAY 已设；这里只兜底）
  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

  # 日志立即刷盘 + 带时间戳，断电不丢尾巴
  export PYTHONUNBUFFERED=1

  echo "[autostart-gs] 启动: python3 -m ground_station.main"
  echo "[autostart-gs] 控制台日志 -> $AUTO_LOG"
  echo "[autostart-gs] === 地面站自启动开始 ==="

  # 全屏启动地面站（默认全屏；调试想要窗口化加 --windowed）
  exec python3 -m ground_station.main
} 2>&1 | tee -a "$AUTO_LOG"
