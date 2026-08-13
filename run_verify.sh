#!/usr/bin/env bash
# 启动全栈 + 端到端验证
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

find /home/ubuntu/zx2026_arena_ws/src -name '*.py' -exec chmod +x {} +

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "zx2026_all.launch" 2>/dev/null
sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_smoke.log 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 60); do
  rostopic list >/dev/null 2>&1 && break
  sleep 0.5
done
for i in $(seq 1 90); do
  N=$(rosnode list 2>/dev/null | wc -l)
  [ "$N" -ge 20 ] && break
  sleep 0.5
done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"

python3 /home/ubuntu/zx2026_arena_ws/verify_run.py

kill $LAUNCH_PID 2>/dev/null
pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "== VERIFY DONE =="
