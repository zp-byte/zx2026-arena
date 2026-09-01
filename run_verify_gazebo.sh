#!/usr/bin/env bash
# 启动 Gazebo 物理后端全栈 + 端到端验证（无头 gzserver，端口 11411）
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "gazebo.launch" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/zx2026_gazebo_smoke.log 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 60); do
  rostopic list >/dev/null 2>&1 && break
  sleep 0.5
done
for i in $(seq 1 120); do
  N=$(rosnode list 2>/dev/null | wc -l)
  [ "$N" -ge 20 ] && break
  sleep 0.5
done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"

# 等 gazebo 真正发布 /clock（gzserver 就绪）
for i in $(seq 1 60); do
  rostopic list 2>/dev/null | grep -q '^/clock' && break
  sleep 0.5
done
echo "clock topic: $(rostopic list 2>/dev/null | grep -c '^/clock')"

python3 /home/ubuntu/zx2026_arena_ws/verify_run.py

kill $LAUNCH_PID 2>/dev/null
pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "roscore.*11411" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
sleep 2
echo "== GAZEBO VERIFY DONE =="
