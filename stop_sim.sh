#!/usr/bin/env bash
# 停止仿真全部进程（roscore/roslaunch/gzserver/gzclient/rviz 及所有节点）
pkill -f 'rosmaster' 2>/dev/null
pkill -f 'roscore' 2>/dev/null
pkill -f 'roslaunch' 2>/dev/null
pkill -f 'gzserver' 2>/dev/null
pkill -f 'gzclient' 2>/dev/null
pkill -f 'rviz' 2>/dev/null
pkill -f 'arena_' 2>/dev/null
pkill -f '_node.py' 2>/dev/null
sleep 2
echo "stopped"
