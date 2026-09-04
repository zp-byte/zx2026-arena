#!/bin/bash
# tools/monitor_wait_state.sh <target_state> [timeout_s]
# 轮询 /zx2026/state 直到目标相位（供 e2e 驱动与人工排障用）
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
target=${1:-DONE}
limit=${2:-420}
t0=$(date +%s)
while :; do
  # rostopic echo 输出 data: "DONE"——去引号后再比较（教训：带引号比较永假）
  st=$(timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null |
       awk '/^data:/ {gsub(/"/, "", $2); print $2; exit}')
  now=$(date +%s)
  echo "$(date +%H:%M:%S) state=${st:-none}"
  if [ "$st" = "$target" ]; then echo "REACHED $target after $((now - t0))s"; exit 0; fi
  if [ $((now - t0)) -ge $limit ]; then echo "TIMEOUT waiting $target"; exit 1; fi
  sleep 5
done
