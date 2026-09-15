#!/usr/bin/env bash
# tools/gz_closure_run.sh — Gazebo 实弹官宣收口 v2（2026-09-15）
#
# v1 教训：gazebo_e2e.sh 的观察循环把 latched 的 /zx2026/score_summary 首次
# 出现当任务终点（首架穿越即发布，landed=False）→ 三局均在 wall+74s、
# 机群 RETURN 半途被截杀——"0 碰撞"是截断假象，scorekeeper/nav_metrics
# 均未及落盘。v2 自带观察循环：只认 done=6/6 或 /zx2026/state=DONE，
# 600s wall 上限（现行栈 sim 局 done≈170-190s，RTF≈1）。
#
# 口径（预登记）：Gazebo 后端无树枝物理——world_builder 只建树干圆柱
# （碰撞）+树冠球（视觉），gazebo_collision_monitor 只查 trunk/bush/fence/
# 机间。本官宣验证全栈（含 lissajous+云记忆，lidar_node 全栈继承）在物理
# 后端的总体表现：col 计数（monitor logwarn "COLLISION #" 带坐标）、
# score、全时长无右删失、NE 无断档。树枝避障本体已由 sim 24-cell 矩阵
# 定谳（c283f46），不在 gz 重证。
#
# 用法: bash ~/zx2026_arena_ws/tools/gz_closure_run.sh
WS=/home/ubuntu/zx2026_arena_ws
CFG=$WS/src/zx2026_common/config/sim_settings.yaml
TS=$(date +%Y%m%d_%H%M%S)
OUT=$WS/run_logs/gz_closure_$TS
mkdir -p "$OUT"
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models
# set -u 须在 ROS setup 之后（profile 脚本读未定义 ROS_DISTRO 会炸）
set -u
SEEDS="42 43 44"

cleanup() {
  pkill -f "rosmaster.*11411" 2>/dev/null
  pkill -f gzserver 2>/dev/null
  pkill -f gzclient 2>/dev/null
  sleep 3
}
restore_seed() {
  sed -i "s/^run_seed: *[0-9]\+/run_seed: 42/" "$CFG"
}
trap 'restore_seed; cleanup' EXIT

for S in $SEEDS; do
  echo "===== gz closure seed=$S ====="
  # 0a) 自愈 exec bit（124209 教训）：UNC 编辑 node 脚本会掉可执行位，
  # roslaunch spawn 静默失败只进 roslaunch-*.log。发车前强制复原。
  find $WS/src -name "*.py" -path "*/scripts/*" -exec chmod +x {} \;
  # 0) 清场点名：零 ROS/GZ 进程才发车（防串局污染）
  N=1
  for i in $(seq 1 30); do
    N=$(pgrep -fc "rosmaster|gzserver|roslaunch" || true)
    [ "$N" = "0" ] && break
    sleep 2
  done
  echo "pre-launch residual procs: $N"
  # 1) seed 补丁（只替换数字，保行尾注释）
  sed -i "s/^run_seed: *[0-9]\+/run_seed: $S/" "$CFG"
  grep "^run_seed:" "$CFG"
  MARKER=/tmp/gz_run_marker_$S
  touch "$MARKER"
  # 2) 发车
  nohup roslaunch arena_world_gazebo gazebo.launch \
    > /tmp/zx2026_gz_e2e.log 2>&1 &
  for i in $(seq 1 120); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
  OK=""
  for i in $(seq 1 60); do
    ST=$(timeout 2 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
    [ "$ST" = '"P2_WAIT"' ] && OK=1 && break
    sleep 1
  done
  if [ -z "$OK" ]; then
    echo "FAIL: never reached P2_WAIT (seed=$S)"
    cp /tmp/zx2026_gz_e2e.log "$OUT/seed${S}_gz.log" 2>/dev/null
    cleanup
    continue
  fi
  echo "P2_WAIT at +${i}s"
  # 活体检查（124209 教训）：UNC 编辑掉 exec bit → scorekeeper 未被 spawn，
  # 三局 score yaml/rosout 全空，roslaunch 报错只进 roslaunch-*.log 不进终端。
  # 判据=scorekeeper 的 publisher 话题在列（init 即注册 /zx2026/score/<i>）。
  # 不能等消息：latched 话题在首次 LANDED/释放 publish 前无内容，echo 必空转
  # （bkbtraaps 三局 FATAL 误报教训）。
  SKALIVE=""
  for i in $(seq 1 10); do
    rostopic list 2>/dev/null | grep -q "^/zx2026/score/0$" && SKALIVE=1 && break
    sleep 1
  done
  if [ -z "$SKALIVE" ]; then
    echo "FATAL: scorekeeper not alive (score/0 topic absent) — abort seed $S"
    cp /tmp/zx2026_gz_e2e.log "$OUT/seed${S}_gz.log" 2>/dev/null
    cleanup
    continue
  fi
  # start 前等 /zx2026/start 服务在列（bkmfoi31i 教训：P2_WAIT 状态早于服务
  # 注册完，早调 rosservice call 会挂死——服务不存在时 rosservice 等待无超时）
  SVCOK=""
  for i in $(seq 1 30); do
    rosservice list 2>/dev/null | grep -q "^/zx2026/start$" && SVCOK=1 && break
    sleep 1
  done
  if [ -z "$SVCOK" ]; then
    echo "FATAL: /zx2026/start service never registered — abort seed $S"
    cp /tmp/zx2026_gz_e2e.log "$OUT/seed${S}_gz.log" 2>/dev/null
    cleanup
    continue
  fi
  rosservice call /zx2026/start
  # 3) 观察循环：done=6/6 或 state=DONE 才停；否则 600s wall 上限
  T0=$(date +%s)
  DONED=0
  for k in $(seq 1 20); do
    sleep 30
    EL=$(( $(date +%s) - T0 ))
    SIM=$(timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -oP 'secs: \K[0-9]+' | head -1)
    ST=$(timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"' | head -1)
    ND=$(for d in 0 1 2 3 4 5; do timeout 2 rostopic echo -n1 /drone_$d/mission/phase 2>/dev/null | grep -c 'data: "DONE"'; done | grep -c 1)
    echo "wall+${EL}s sim=${SIM}s state=$ST done=$ND/6"
    if [ "${ND:-0}" -ge 6 ] || [ "$ST" = '"DONE"' ]; then DONED=1; break; fi
  done
  [ "$DONED" = "1" ] || echo "WARNING: window exhausted before DONE (right-censor)"
  # 4) 局末 per-drone 快照
  for d in 0 1 2 3 4 5; do
    P=$(timeout 3 rostopic echo -n1 /drone_$d/mission/phase 2>/dev/null | grep -o 'data: "[A-Z_]*"' | head -1)
    SC=$(timeout 3 rostopic echo -n1 /zx2026/score/$d 2>/dev/null | grep -E "score:" | head -1 | tr -d ' ')
    echo "drone$d $P $SC"
  done
  SUM=$(timeout 3 rostopic echo -n1 /zx2026/score_summary 2>/dev/null | grep data | head -1)
  echo "score_summary: ${SUM:-none}"
  sleep 3   # 让 nav_metrics final 落盘
  # 5) 归档
  cp /tmp/zx2026_gz_e2e.log "$OUT/seed${S}_gz.log" 2>/dev/null
  if [ -e ~/.ros/log/latest/rosout.log ]; then
    cp ~/.ros/log/latest/rosout.log "$OUT/seed${S}_rosout.log"
    grep -a "COLLISION #" "$OUT/seed${S}_rosout.log" \
      > "$OUT/seed${S}_collisions.txt" || true
    grep -a "NAV_METRICS_SUMMARY" "$OUT/seed${S}_rosout.log" \
      > "$OUT/seed${S}_nav_summary.txt" 2>/dev/null || true
  else
    echo "(no rosout.log)" > "$OUT/seed${S}_collisions.txt"
  fi
  cp $WS/run_logs/nav_metrics_latest.yaml "$OUT/seed${S}_nav_metrics.yaml" 2>/dev/null
  find /tmp -maxdepth 1 -name "zx2026_score_*.yaml" -newer "$MARKER" \
    -exec cp {} "$OUT/" \;
  echo "collisions seed=$S: $(grep -c . "$OUT/seed${S}_collisions.txt" 2>/dev/null || true)"
  cleanup
done

echo "===== ALL DONE ====="
echo "output: $OUT"
