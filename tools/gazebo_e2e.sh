#!/usr/bin/env bash
# Gazebo 后端端到端：启动 → P2_WAIT → /zx2026/start → 观测 → 计分
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

find /home/ubuntu/zx2026_arena_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/zx2026_gz_e2e.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done

# 等 fleet_ready → P2_WAIT（最多 60s）
echo "== waiting P2_WAIT =="
OK=""
for i in $(seq 1 60); do
  ST=$(timeout 2 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
  [ "$ST" = '"P2_WAIT"' ] && OK=1 && break
  sleep 1
done
[ -n "$OK" ] || { echo "FAIL: never reached P2_WAIT"; pkill -f "rosmaster.*11411"; exit 1; }
echo "P2_WAIT at +${i}s"

echo "== /zx2026/start =="
rosservice call /zx2026/start

echo "== observe up to 480s wall (every 30s) =="
echo "  sim_t drone0_phase  drone0_pos  done_count"
T0=$(date +%s)
SUMMARY=""
for k in $(seq 1 16); do
  sleep 30
  NOW=$(date +%s); EL=$((NOW-T0))
  SIM=$(timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -oP 'secs: \K[0-9]+' | head -1)
  P0=$(timeout 3 rostopic echo -n1 /drone_0/mission/phase 2>/dev/null | grep -o 'data: "[A-Z_]*"' | head -1)
  POS=$(timeout 3 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' ')
  ND=$(for i in 0 1 2 3 4 5; do timeout 2 rostopic echo -n1 /drone_$i/mission/phase 2>/dev/null | grep -c 'DONE'; done | grep -c 1)
  echo "wall+${EL}s sim=${SIM}s drone0=$P0 pos=$POS done=$ND/6"
  # 计分摘要出现即提前结束
  SUMMARY=$(timeout 3 rostopic echo -n1 /zx2026/score_summary 2>/dev/null | grep data | head -1)
  [ -n "$SUMMARY" ] && break
done

echo "== per-drone final phases =="
for i in 0 1 2 3 4 5; do
  P=$(timeout 3 rostopic echo -n1 /drone_$i/mission/phase 2>/dev/null | grep -o 'data: "[A-Z_]*"' | head -1)
  S=$(timeout 3 rostopic echo -n1 /zx2026/score/$i 2>/dev/null | grep -E "score:|match" | tr '\n' ' ')
  echo "drone$i $P  $S"
done
echo "== score summary =="
[ -n "$SUMMARY" ] && echo "score_summary: $SUMMARY" || echo "score_summary: (none yet)"
echo "== score yaml (tail) =="
tail -20 /tmp/zx2026_score_*.yaml 2>/dev/null || echo "(no yaml yet)"

echo "== path topics (new plugin) =="
for i in 0 1 2 3 4 5; do
  N=$(timeout 3 rostopic echo -n1 /drone_$i/path 2>/dev/null | grep -c "poses: -")
  echo "drone$i path poses: $N"
done
echo "md5/Path errors in log: $(grep -c 'md5sum\|wants topic /drone_._/odom to have datatype' /tmp/zx2026_gz_e2e.log 2>/dev/null)"

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null
echo "== GAZEBO E2E DONE =="
