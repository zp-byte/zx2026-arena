#!/usr/bin/env bash
# 验证 gazebo_collision_monitor 修复：跑完整 Gazebo 任务，盯 6 机 phase + 碰撞恢复
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0

find /home/ubuntu/zx2026_arena_ws/src -name '*.py' -exec chmod +x {} + 2>/dev/null
bash /home/ubuntu/zx2026_arena_ws/kill_all.sh >/dev/null 2>&1
sleep 2

setsid nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/zx2026_gazebo_fix.log 2>&1 < /dev/null &

# 等节点起来
for i in $(seq 1 120); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"
rosnode list 2>/dev/null | grep -c gazebo_collision_monitor | xargs echo "collision_monitor:"

# 等 P2_WAIT 后开始（轮询直到出现 P2_WAIT 再 call start；启动瞬间可能还在 P1_INIT）
ST=""
STARTED=0
for i in $(seq 1 90); do
  ST=$(timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
  echo "wait stage: $ST"
  if [ "$ST" = '"P2_WAIT"' ]; then
    timeout 5 rosservice call /zx2026/start >/dev/null 2>&1
    echo "started"
    STARTED=1
    break
  fi
  sleep 2
done
[ "$STARTED" = "0" ] && echo "WARN: never reached P2_WAIT, forcing start" && timeout 5 rosservice call /zx2026/start >/dev/null 2>&1

# 轮询：全部 DONE 或超时 900s
T0=$(date +%s)
COLS=0
while true; do
  NOW=$(date +%s)
  EL=$((NOW - T0))
  ST=$(timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
  PH=""
  for i in 0 1 2 3 4 5; do
    P=$(timeout 2 rostopic echo -n1 /drone_$i/mission/phase 2>/dev/null | grep -o '"DONE"' | head -1)
    PH="$PH$i:${P:-...} "
  done
  # 累计碰撞次数（监控节点日志）
  C=$(grep -ac "COLLISION" /tmp/zx2026_gazebo_fix.log 2>/dev/null)
  echo "[${EL}s] $ST | phases: $PH | collisions: $C"
  COLS=$C
  case "$PH" in
    *"0:DONE"*"1:DONE"*"2:DONE"*"3:DONE"*"4:DONE"*"5:DONE"*) echo "== ALL 6 DONE =="; break ;;
  esac
  if [ "$EL" -gt 900 ]; then echo "== TIMEOUT 900s =="; break; fi
  sleep 30
done

echo "--- 碰撞恢复日志 ---"
grep -a "COLLISION\|frozen\|drone [0-9] COLLISION" /tmp/zx2026_gazebo_fix.log 2>/dev/null | tail -20 | cut -c1-160
echo "--- 最新 score yaml ---"
ls -t /tmp/zx2026_score_*.yaml 2>/dev/null | head -1 | xargs -I{} cat {}
echo "== VERIFY FIX DONE =="
