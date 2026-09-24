#!/bin/sh
# FR-4.x E2E：hub(空管) + mavlink bridge + 两台假 PX4 全链路
# 预期：PROX_WARN/CRIT(对飞) + land 闭环 + mission 上传闭环
set -u
cd /home/ubuntu/zx2026_arena_ws/tools/gcs || exit 1
OUT=/mnt/c/Users/24882/Desktop/_gcs_mavlink_e2e.txt
: > "$OUT"

pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_mavlink_bridge.py 2>/dev/null
sleep 1

python3 gcs_hub.py --view log --ids 0,1 \
  --status-file /tmp/gcs_ml_status.json --logdir /tmp >/tmp/gcs_ml_hub.log 2>&1 &
HUB_PID=$!
sleep 1
python3 gcs_mavlink_bridge.py --connect udpin:127.0.0.1:14555 \
  --map "1:0,2:1" --enu-lat 30.0 --enu-lon 120.0 \
  --hub 127.0.0.1:9870 >/tmp/gcs_ml_bridge.log 2>&1 &
BR_PID=$!
sleep 2

{
echo "== fake PX4 x2 全链路（20s）=="
python3 fake_mavlink_test.py 2>&1
echo
echo "== bridge log =="
cat /tmp/gcs_ml_bridge.log
echo
echo "== hub 事件流 =="
cat /tmp/gcs_ml_hub.log
echo
echo "== E2E_DONE =="
} >> "$OUT" 2>&1

kill "$HUB_PID" "$BR_PID" 2>/dev/null
exit 0
