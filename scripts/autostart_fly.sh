#!/usr/bin/env bash
set -x

LOG_DIR="$HOME/fly_logs"
mkdir -p "$LOG_DIR"
AUTO_LOG="$LOG_DIR/autostart_$(date +%Y%m%d_%H%M%S).log"

{
  echo "[autostart] started at $(date)"
  echo "[autostart] user=$(whoami)"
  echo "[autostart] pwd=$(pwd)"
  echo "[autostart] DISPLAY=${DISPLAY:-}"

  sleep 8
  echo "[autostart] after sleep at $(date)"

  cd "$HOME/kian_ws" || exit 1

  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

  ./scripts/run_test.sh demo3.launch.py
} 2>&1 | tee -a "$AUTO_LOG"