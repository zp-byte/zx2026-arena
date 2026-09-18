#!/bin/bash
# tools/gcs/bench_check.sh — no-prop bench 一键核对（TODO③ 机侧段，2026-09-18）。
#
# 前置：机载 Jetson 上 faster_lio + mavros + px4ctrl 已起，**确认无螺旋桨**。
# 用法（机载，source ~/Diff-planner/devel/setup.sh 后）：
#   bash bench_check.sh --id 0
# 地面站段（agent/hub 通路）另跑：tools/gcs/fake_agent_test.py + gcs_ops preflight。
set -u
ID=0
while [ $# -gt 0 ]; do
  case "$1" in
    --id) ID="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done
VC="/drone_$ID/vel_cmd"
fail=0

chk() { # chk <名> <ok(0/1)> <备注>
  if [ "$2" -eq 0 ]; then echo "   [ok] $1"; else echo "   [FAIL] $1（$3）"; fail=1; fi
}

echo "== 1) 话题/服务在列 =="
rostopic list >/tmp/bench_topics.$$.txt 2>&1
for t in /Odometry /mavros/state /mavros/battery; do
  grep -qx "$t" /tmp/bench_topics.$$.txt; chk "topic $t" $? "未见——查对应节点"
done
rosservice list 2>/dev/null | grep -q "^/px4ctrl/takeoff_land$"
chk "service /px4ctrl/takeoff_land" $? "px4ctrl 未起"

echo "== 2) LIO 频率（>=20Hz 判活）=="
hz_out=$(timeout 8 rostopic hz /Odometry 2>&1 | grep -m1 average)
echo "   $hz_out"
echo "$hz_out" | grep -qE 'average rate: ([2-9][0-9]|[0-9]{3})' \
  || chk "/Odometry >=20Hz" 1 "$hz_out"

echo "== 3) mavros FCU 链路 =="
st=$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -m1 connected)
echo "   $st"
echo "$st" | grep -q "True"; chk "mavros connected" $? "FCU 未连（查串口/UDP）"

echo "== 4) vel_bridge 无桨链路（RC 保持 HOVER/MANUAL 档！）=="
python3 "$(dirname "$0")/vel_bridge.py" --id "$ID" >/tmp/bench_bridge.$$.log 2>&1 &
bp=$!; sleep 2
# 先挂 echo 再注指令——echo 若起在 pub 死后，抓到的是 HOLD 的 0 速（假阴性）
timeout 8 rostopic echo -n1 /position_cmd >/tmp/bench_pc.$$.txt 2>/dev/null &
ep=$!; sleep 0.5
# 注 3s 0.5m/s 前向速度指令（无桨机体不动，只验数据链）
timeout 3 rostopic pub -r 20 "$VC" geometry_msgs/Twist \
  '{linear: {x: 0.5}}' >/dev/null 2>&1
wait $ep; pc=$(cat /tmp/bench_pc.$$.txt)
kill $bp 2>/dev/null; wait $bp 2>/dev/null
echo "$pc" | grep -q "x: 0.5"; chk "/position_cmd velocity.x=0.5 直传" $?
echo "$pc" | grep -q "^yaw";  chk "/position_cmd yaw 字段在" $?
grep -q "IDLE -> ACTIVE" /tmp/bench_bridge.$$.log \
  && echo "   [ok] 桥状态机 ACTIVE→HOLD 日志在" \
  || { echo "   [FAIL] 桥状态机日志缺"; fail=1; }
echo "-- 桥日志尾："; tail -3 /tmp/bench_bridge.$$.log | sed 's/^/   /'

rm -f /tmp/bench_topics.$$.txt /tmp/bench_bridge.$$.log
echo; [ $fail -eq 0 ] && echo "bench_check PASS" || echo "bench_check FAIL（上列 FAIL 项清零后再实弹）"
exit $fail
