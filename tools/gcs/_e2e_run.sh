#!/bin/sh
# 合并后离线 E2E：hub(--scene) + fake 两测 + 面板 selftest + FR 全路径 feeder
set -u
cd /home/ubuntu/zx2026_arena_ws/tools/gcs || exit 1
OUT=/mnt/c/Users/24882/Desktop/_gcs_e2e.txt
: > "$OUT"
SCENE=/home/ubuntu/zx2026_arena_ws/src/zx2026_common/config/scene_topology.yaml

# 清场（防残留 hub/panel 占 9870/读旧 status）
pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_panel.py 2>/dev/null
sleep 1

python3 gcs_hub.py --view log --ids 0,1,2,3,4,5 --scene "$SCENE" \
  --status-file /tmp/gcs_e2e_status.json --logdir /tmp >/tmp/gcs_e2e_hub.log 2>&1 &
HUB_PID=$!
sleep 2

{
echo "== [1] fake_agent_test（预期 STALL/STALL_CLEAR/LINK_LOST）=="
python3 fake_agent_test.py 2>&1 | tail -2
echo
echo "== [2] fake_plandead_test（预期 PLANNER_DEAD 边沿）=="
python3 fake_plandead_test.py 2>&1 | tail -2
echo
echo "== [3] 面板 selftest + FR feeder（联动）=="
QT_QPA_PLATFORM=offscreen python3 gcs_panel.py --profile profile_sim.yaml \
  --status-file /tmp/gcs_e2e_status.json --selftest >/tmp/gcs_e2e_panel.log 2>&1 &
PANEL_PID=$!
sleep 2
python3 _e2e_feeder.py 2>&1 | tail -3
# 等面板自行退出（全终态），最多 30s
i=0
while [ $i -lt 30 ]; do
  kill -0 "$PANEL_PID" 2>/dev/null || break
  i=$((i + 1)); sleep 1
done
[ $i -ge 30 ] && { echo "PANEL_TIMEOUT_KILL"; kill "$PANEL_PID" 2>/dev/null; }
wait "$PANEL_PID" 2>/dev/null
PANEL_RC=$?
echo "panel rc=$PANEL_RC"
echo
echo "== hub 事件流全量 =="
cat /tmp/gcs_e2e_hub.log
echo
echo "== 面板输出 =="
cat /tmp/gcs_e2e_panel.log
echo
echo "== E2E_DONE =="
} >> "$OUT" 2>&1

kill "$HUB_PID" 2>/dev/null
exit 0
