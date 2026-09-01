#!/usr/bin/env bash
# 分析当前比赛进度快照：阶段 / 各机任务状态 / 位置 / 计分
export ROS_MASTER_URI=http://127.0.0.1:11411
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

echo "=== 阶段 state ==="
timeout 4 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -E "data" | head -2
echo "=== 世界时钟 ==="
timeout 3 rostopic echo -n1 /clock/clock 2>/dev/null | grep -E "secs" | head -1

echo "=== 各机任务状态 (/zx2026/mission/i) ==="
for i in 0 1 2 3 4 5; do
  M=$(timeout 3 rostopic echo -n1 /zx2026/mission/$i 2>/dev/null | grep -E "status|phase|done|matched|drop" | head -4 | tr '\n' ' ')
  echo "drone $i: $M"
done

echo "=== 各机位置 ==="
for i in 0 1 2 3 4 5; do
  P=$(timeout 3 rostopic echo -n1 /drone_$i/odom/pose/pose/position 2>/dev/null | grep -E "x:|y:|z:" | head -3 | tr '\n' ' ')
  echo "drone $i: $P"
done

echo "=== 计分 ==="
timeout 4 rostopic echo -n1 /zx2026/score_summary 2>&1 | head -20

echo "=== 碰撞计数 ==="
timeout 3 rostopic echo -n1 /zx2026/collisions 2>&1 | head -5
echo "=== DONE ==="
