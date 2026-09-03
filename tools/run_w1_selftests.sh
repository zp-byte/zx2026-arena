#!/bin/bash
# W1 全量 selftest 套件（新 + 存量回归）
source /opt/ros/noetic/setup.bash
source ~/zx2026_arena_ws/devel/setup.bash
cd ~/zx2026_arena_ws
for t in w1_swarm via_slots avoid dam de nstop p4 plan rescue veto; do
  echo "== $t =="
  python3 "tools/${t}_selftest.py" 2>&1 | tail -4
done
