#!/usr/bin/env bash

echo "[stop] stopping fly processes..."

pkill -INT -f "ros2 bag record"
sleep 1
pkill -INT -f "ros2 launch my_launch demo3.launch.py"
sleep 1
pkill -INT -f "scripts/run_test.sh"
sleep 2

echo "[stop] remaining related processes:"
pgrep -af "run_test.sh|ros2 bag record|ros2 launch my_launch demo3.launch.py" || true

echo "[stop] done"