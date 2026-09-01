#!/usr/bin/env bash
# 启动全栈（后台常驻，不自动验证、不自动杀，供后续交互/可视化使用）
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

find /home/ubuntu/zx2026_arena_ws/src -name '*.py' -exec chmod +x {} +

pkill -f 'rosmaster.*11411' 2>/dev/null
pkill -f 'zx2026_all.launch' 2>/dev/null
sleep 1

setsid nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_smoke.log 2>&1 < /dev/null &
echo "launch pid $!"
