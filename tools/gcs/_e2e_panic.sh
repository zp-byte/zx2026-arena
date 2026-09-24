#!/bin/sh
# P0-② panic 双轨 E2E：富机 d0 走模板（假 ssh）、贫机 d1 走 bridge MAVLink land
# 预期：d0 panic OK + d1 panic OK + fake IDLE 收到 LAND seen → PANIC COMPLETE
set -u
cd /home/ubuntu/zx2026_arena_ws/tools/gcs || exit 1
OUT=/mnt/c/Users/24882/Desktop/_gcs_panic_e2e.txt
: > "$OUT"

pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_mavlink_bridge.py 2>/dev/null
pkill -f fake_mavlink_idle.py 2>/dev/null
sleep 1

python3 gcs_hub.py --view log --ids 0,1 \
  --status-file /tmp/gcs_ml_status.json --logdir /tmp >/tmp/gcs_pn_hub.log 2>&1 &
sleep 1
python3 gcs_mavlink_bridge.py --connect udpin:127.0.0.1:14555 \
  --map "2:1" --enu-lat 30.0 --enu-lon 120.0 \
  --hub 127.0.0.1:9870 >/tmp/gcs_pn_bridge.log 2>&1 &
sleep 2
python3 fake_mavlink_idle.py --sysids 2 >/tmp/gcs_pn_idle.log 2>&1 &
sleep 2

{
echo "== panic 双轨（d0 模板 / d1 MAVLink）=="
timeout 40 python3 gcs_ops.py --profile _test_panic_profile.yaml panic
rc=$?
echo "ops_exit=$rc"
echo
sleep 3   # 等 fake 的 0.1s 抽干循环轮到 recv+print（cat 太早=假阴性）
echo "== idle fake log =="
cat /tmp/gcs_pn_idle.log
echo
echo "== bridge log（tail 5）=="
tail -5 /tmp/gcs_pn_bridge.log
echo
echo "== PANIC_E2E_DONE =="
} >> "$OUT" 2>&1

pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_mavlink_bridge.py 2>/dev/null
pkill -f fake_mavlink_idle.py 2>/dev/null
exit 0
