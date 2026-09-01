#!/usr/bin/env bash
# 临时：群集端到端验证（含 flock 指标采样），等价 run_verify.sh 但跑 flock_obs.py
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

find /home/ubuntu/zx2026_arena_ws/src -name '*.py' -exec chmod +x {} +

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "zx2026_all.launch" 2>/dev/null
sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_flock.log 2>&1 &
for i in $(seq 1 90); do
  N=$(rosnode list 2>/dev/null | wc -l)
  [ "$N" -ge 20 ] && break
  sleep 0.5
done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"

python3 /home/ubuntu/zx2026_arena_ws/flock_obs.py

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "== FLOCK RUN DONE =="
