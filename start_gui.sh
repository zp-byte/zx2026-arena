#!/usr/bin/env bash
# 启动 Gazebo 物理后端 + RViz（GUI，WSLg 显示）+ 完整任务栈
# 之后: rosservice call /zx2026/start 开始比赛
cd /home/ubuntu/zx2026_arena_ws || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1

find src -name '*.py' -exec chmod +x {} +

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "gazebo.launch" 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
sleep 2

roslaunch arena_world_gazebo gazebo.launch viewer:=true
echo "== gazebo+rviz exited rc=$? =="
