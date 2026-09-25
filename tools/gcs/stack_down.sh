#!/bin/bash
# stack_down.sh — GCS+sim 全场清点（桌面 gcs_down.bat 调用）
# 纪律：pkill 模式词必须住脚本文件——外层命令行含词=自杀（三犯教训）
source /opt/ros/noetic/setup.bash 2>/dev/null
for pat in "roslaunc[h]" "rvizi[c]" "gzserve[r]" "gzclien[t]" "rosmaste[r]" \
           "gcs_age[n]" "gcs_hu[b]" "gcs_mavlink_bridg[e]" "gcs_pane[l]" \
           "executor_" "nav_nod[e]" "stage_controlle[r]" "world_nod[e]" \
           "scorekeepe[r]" "fleet_nod[e]" "comm_mode[l]" "tag_detecto[r]" \
           "collision_monito[r]"; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2
pkill -9 -f "roslaunc[h]" 2>/dev/null
pkill -9 -f "rosmaste[r]" 2>/dev/null
sleep 1
echo "CLEANED: master=$(pgrep -c rosmaste[r] || true) gzserver=$(pgrep -c gzserve[r] || true) hub=$(pgrep -c gcs_hu[b] || true)"
