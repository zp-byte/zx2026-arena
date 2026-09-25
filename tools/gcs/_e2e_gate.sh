#!/bin/sh
# P1 闸门 E2E：gate 覆盖率自动判据（PASS/HOLD 两态）+ deploy（贫机 mission+takeoff）
# 前置断言依赖：feeder 喂 d0 grid（覆盖率爬升）+ d1 fc=AUTO；fake PX4=sysid2(→d1)
set -u
cd /home/ubuntu/zx2026_arena_ws/tools/gcs || exit 1
OUT=/mnt/c/Users/24882/Desktop/_gcs_gate_e2e.txt
: > "$OUT"

pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_mavlink_bridge.py 2>/dev/null
pkill -f fake_mavlink_idle.py 2>/dev/null; pkill -f _e2e_gate_feeder.py 2>/dev/null
sleep 1

python3 gcs_hub.py --view log --ids 0,1 \
  --status-file /tmp/gcs_ml_status.json --logdir /tmp >/tmp/gcs_gd_hub.log 2>&1 &
sleep 1
python3 gcs_mavlink_bridge.py --connect udpin:127.0.0.1:14555 \
  --map "2:1" --enu-lat 30.0 --enu-lon 120.0 \
  --hub 127.0.0.1:9870 >/tmp/gcs_gd_bridge.log 2>&1 &
sleep 2
python3 fake_mavlink_idle.py --sysids 2 >/tmp/gcs_gd_idle.log 2>&1 &
sleep 1
python3 _e2e_gate_feeder.py >/tmp/gcs_gd_feeder.log 2>&1 &
sleep 6   # 等覆盖率爬过 0.3

{
echo "== recon（强机 d0 takeoff+trigger；d1 贫机不出）=="
python3 gcs_ops.py --profile _test_panic_profile.yaml recon
echo "recon_exit=$?"
echo
echo "== gate PASS（阈值 0.3 < 已扫 ~0.3+）=="
python3 gcs_ops.py --profile _test_panic_profile.yaml gate --auto-cover 0.3
echo "gate_exit=$?"
echo
echo "== gate HOLD（阈值 0.9 > 0.5 封顶）=="
python3 gcs_ops.py --profile _test_panic_profile.yaml gate --auto-cover 0.9
echo "hold_exit=$?"
echo
echo "== deploy --yes --no-monitor（param装订+mission+takeoff）=="
python3 gcs_ops.py --profile _test_panic_profile.yaml deploy --yes --no-monitor
echo "deploy_exit=$?"
sleep 3   # 让开 fake 收包窗
echo
echo "== phase 合成断言（REACHED→DONE 锁）=="
python3 - <<'PYEOF'
import json
s = json.load(open('/tmp/gcs_ml_status.json'))
d = (s.get('drones') or {}).get('1') or {}
ph, rk, fc = d.get('phase'), d.get('rtk'), d.get('fc')
print('d1 phase=%s rtk=%s fc=%s' % (ph, rk, fc))
print('PHASE_ASSERT=' + ('PASS' if (ph == 'DONE' and rk == 'RTK_FIXED'
                                    and fc == 'AUTO') else 'FAIL'))
PYEOF
echo
echo "== idle fake log =="
cat /tmp/gcs_gd_idle.log
echo
echo "== hub grid 快照抽查 =="
python3 -c "import json; s=json.load(open('/tmp/gcs_ml_status.json')); print('grids keys:', sorted((s.get('grids') or {}).keys()))"
echo "== GATE_E2E_DONE =="
} >> "$OUT" 2>&1

pkill -f gcs_hub.py 2>/dev/null; pkill -f gcs_mavlink_bridge.py 2>/dev/null
pkill -f fake_mavlink_idle.py 2>/dev/null; pkill -f _e2e_gate_feeder.py 2>/dev/null
exit 0
