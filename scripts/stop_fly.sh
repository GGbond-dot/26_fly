#!/usr/bin/env bash
# 停止自启/试飞的所有进程（消防自启 + 手动 run_test 都覆盖）
# ⚠ launch 名跟着 autostart_fly.sh 的 LAUNCH_PKG/LAUNCH_FILE 走，改一处要同步改另一处。

LOG_DIR="${FLY_LOG_DIR:-$HOME/fly_logs}"
PUSH_USER="${FLY_LOG_PUSH_USER:-kian}"
PUSH_HOST="${FLY_LOG_PUSH_HOST:-192.168.10.116}"
PUSH_DIR="${FLY_LOG_PUSH_DIR:-/home/kian/kian_26fly/fly_log}"
PUSH_TIMEOUT="${FLY_LOG_PUSH_TIMEOUT:-8}"

find_latest_autostart_log() {
  local latest=""
  local file
  for file in "$LOG_DIR"/autostart_*.log; do
    [ -e "$file" ] || continue
    if [ -z "$latest" ] || [ "$file" -nt "$latest" ]; then
      latest="$file"
    fi
  done
  printf '%s\n' "$latest"
}

collect_latest_fly_logs() {
  local autostart_log="$1"
  local base ts bundle_parent bundle_dir archive
  base="$(basename "$autostart_log")"
  ts="${base#autostart_}"
  ts="${ts%.log}"

  bundle_parent="/tmp/fly_log_collect_${ts}_$$"
  bundle_dir="$bundle_parent/fly_log_${ts}"
  archive="$LOG_DIR/fly_log_${ts}.tar.gz"

  mkdir -p "$bundle_dir"
  cp -a "$autostart_log" "$bundle_dir/"

  if [ -f "$LOG_DIR/run_${ts}.log" ]; then
    cp -a "$LOG_DIR/run_${ts}.log" "$bundle_dir/"
  else
    echo "[stop] warning: missing $LOG_DIR/run_${ts}.log" >&2
  fi

  if [ -d "$LOG_DIR/bag_${ts}" ]; then
    cp -a "$LOG_DIR/bag_${ts}" "$bundle_dir/"
  else
    echo "[stop] warning: missing $LOG_DIR/bag_${ts}" >&2
  fi

  {
    echo "collected_at=$(date --iso-8601=seconds)"
    echo "source_log_dir=$LOG_DIR"
    echo "autostart_log=$autostart_log"
    echo "run_log=$LOG_DIR/run_${ts}.log"
    echo "bag_dir=$LOG_DIR/bag_${ts}"
    echo "target=${PUSH_USER}@${PUSH_HOST}:${PUSH_DIR}"
  } > "$bundle_dir/manifest.txt"

  tar -czf "$archive" -C "$bundle_parent" "$(basename "$bundle_dir")"
  printf '%s\n' "$archive"
}

push_archive_to_pc() {
  local archive="$1"
  local remote_host="${FLY_LOG_PUSH_REMOTE:-${PUSH_USER}@${PUSH_HOST}}"
  local remote_dir="$PUSH_DIR"

  if [ -n "${FLY_LOG_PUSH_TARGET:-}" ]; then
    remote_host="${FLY_LOG_PUSH_TARGET%%:*}"
    remote_dir="${FLY_LOG_PUSH_TARGET#*:}"
  fi

  if [ "$remote_host" = "$remote_dir" ]; then
    echo "[stop] skip push: bad FLY_LOG_PUSH_TARGET=${FLY_LOG_PUSH_TARGET:-}"
    echo "[stop] local archive: $archive"
    return 1
  fi

  echo "[stop] pushing archive to $remote_host:$remote_dir"
  if ssh -o ConnectTimeout="$PUSH_TIMEOUT" "$remote_host" "mkdir -p \"$remote_dir\"" \
      && scp -o ConnectTimeout="$PUSH_TIMEOUT" "$archive" "$remote_host:$remote_dir/"; then
    echo "[stop] pushed: $remote_host:$remote_dir/$(basename "$archive")"
  else
    echo "[stop] push failed; local archive kept at: $archive"
    return 1
  fi
}

echo "[stop] stopping fly processes..."

# 与 autostart_fly.sh 保持同一套 env 默认值：默认值若改了，这里自动跟着匹配。
# ⚠ 改 autostart_fly.sh 的 LAUNCH_PKG/LAUNCH_FILE 时，务必同步本处（两脚本要一致）。
LAUNCH_PKG="${AUTOSTART_PKG:-my_launch}"
LAUNCH_FILE="${AUTOSTART_LAUNCH:-fire_full.launch.py}"

pkill -INT -f "ros2 bag record"
sleep 1
# 当前自启的消防 launch（按 env 默认值匹配）
pkill -INT -f "ros2 launch $LAUNCH_PKG $LAUNCH_FILE"
# 兜底：旧的盘点/搬运/植保 launch 名（历史自启遗留）
pkill -INT -f "ros2 launch my_launch inventory_full.launch.py"
pkill -INT -f "ros2 launch my_launch spray_basic.launch.py"
pkill -INT -f "ros2 launch my_launch demo3.launch.py"
sleep 1
pkill -INT -f "scripts/autostart_fly.sh"
pkill -INT -f "scripts/run_test.sh"
sleep 2

# 仍残留的 ros2 launch 子进程（SIGINT 没收住时）补一刀
pkill -INT -f "ros2 launch $LAUNCH_PKG $LAUNCH_FILE" || true

echo "[stop] remaining related processes:"
pgrep -af "autostart_fly.sh|run_test.sh|ros2 bag record|ros2 launch $LAUNCH_PKG|ros2 launch my_launch" || true

echo "[stop] collecting latest autostart logs..."
latest_autostart_log="$(find_latest_autostart_log)"
if [ -n "$latest_autostart_log" ]; then
  archive_path="$(collect_latest_fly_logs "$latest_autostart_log")"
  echo "[stop] archive: $archive_path"
  push_archive_to_pc "$archive_path" || true
else
  echo "[stop] no autostart_*.log found in $LOG_DIR"
fi

echo "[stop] done"
