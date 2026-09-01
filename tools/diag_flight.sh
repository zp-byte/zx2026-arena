#!/usr/bin/env bash
# 深挖每架无人机状态：完整 mission 字段 + 速度(判断是否在动) + 位置
export ROS_MASTER_URI=http://127.0.0.1:11411
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

echo "=== 阶段/时钟 ==="
timeout 4 rostopic echo -n1 /zx2026/state 2>/dev/null | grep data | head -1
timeout 3 rostopic echo -n1 /clock/clock 2>/dev/null | grep -E "secs" | head -1

for i in 0 1 2 3 4 5; do
  echo "=== drone $i ==="
  echo "-- mission 完整 --"
  timeout 3 rostopic echo -n1 /zx2026/mission/$i 2>/dev/null | grep -vE "^---|^header|^  seq|^  stamp|^  frame" | head -25
  echo "-- odom 位置+速度 --"
  timeout 3 rostopic echo -n1 /drone_$i/odom 2>/dev/null | grep -E "x:|y:|z:|linear:" -A1 | grep -vE "linear:|^\-\-|^header|^  seq|^  stamp|^  frame" | head -10
done

echo "=== 已计分条目 ==="
timeout 4 rostopic echo -n1 /zx2026/score_summary 2>/dev/null | grep -oE "drone=[0-9].*" | head -8
echo "=== DONE ==="
