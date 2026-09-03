#!/bin/bash
# 全史碰撞事件统一提取 → /tmp/all_collisions.txt
BASE=/home/ubuntu/.ros/log
OUT=/tmp/all_collisions.txt
> $OUT
for d in $BASE/*/; do
  run=$(basename $d)
  [ "$run" = "latest" ] && continue
  if ls $d/gazebo_collision_monitor-*.log >/dev/null 2>&1; then
    grep -a -h 'COLLISION #' $d/gazebo_collision_monitor-*.log 2>/dev/null | while IFS= read -r line; do
      echo "gz|$run|$line"
    done
  else
    grep -a -h 'collision marker at' $d/*.log 2>/dev/null | while IFS= read -r line; do
      echo "py|$run|$line"
    done
  fi
done > $OUT
wc -l $OUT
head -3 $OUT
