#!/usr/bin/env bash
# 每架无人机完整状态：phase / crossed / at_drop / payload / match / collision / score
export ROS_MASTER_URI=http://127.0.0.1:11411
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash

echo "=== 阶段/时钟 ==="
timeout 4 rostopic echo -n1 /zx2026/state 2>/dev/null | grep data | head -1
timeout 3 rostopic echo -n1 /clock/clock 2>/dev/null | grep -E "secs" | head -1

for i in 0 1 2 3 4 5; do
  PH=$(timeout 3 rostopic echo -n1 /drone_$i/mission/phase 2>/dev/null | grep -E "data|phase" | head -2 | tr '\n' ' ')
  CR=$(timeout 3 rostopic echo -n1 /drone_$i/mission/crossed_zone 2>/dev/null | grep data | head -1)
  AD=$(timeout 3 rostopic echo -n1 /drone_$i/mission/at_drop 2>/dev/null | grep data | head -1)
  PD=$(timeout 3 rostopic echo -n1 /drone_$i/payload/done 2>/dev/null | grep data | head -1)
  MR=$(timeout 3 rostopic echo -n1 /drone_$i/match/result 2>/dev/null | grep -E "data|match|correct" | head -2 | tr '\n' ' ')
  CO=$(timeout 3 rostopic echo -n1 /drone_$i/collision 2>/dev/null | grep data | head -1)
  SC=$(timeout 3 rostopic echo -n1 /zx2026/score/$i 2>/dev/null | grep -E "score|correct|match|crossed" | head -5 | tr '\n' ' ')
  echo "--- drone $i ---"
  echo "  phase:     $PH"
  echo "  crossed:   $CR"
  echo "  at_drop:   $AD"
  echo "  payload:   $PD"
  echo "  match:     $MR"
  echo "  collision: $CO"
  echo "  score:     $SC"
done
echo "=== DONE ==="
