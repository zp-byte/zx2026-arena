#!/bin/bash
# tools/monitor_e2e_check.sh — fleet_monitor 实弹验证：
# 起栈（python 后端）→ 挂监测器（--noline --record）→ 开赛 → 等 DONE →
# 采样面板/存证 → 拆栈。全程镜像 run_verify.sh 的起止纪律。
set -x
cd /home/ubuntu/zx2026_arena_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1

# 预清场（同 run_verify.sh）
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "zx2026_all.launch" 2>/dev/null; sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_mon_smoke.log 2>&1 &
LPID=$!

# 就绪 1: rostopic list（<=30s）
ok=0
for i in $(seq 1 60); do
  if timeout 3 rostopic list >/tmp/mon_topics.txt 2>/dev/null; then ok=1; break; fi
  sleep 0.5
done
if [ "$ok" != 1 ]; then echo "FAIL: topics not up"; kill $LPID 2>/dev/null; exit 1; fi

# 就绪 2: 节点数 >= 20（<=45s）
ok=0
for i in $(seq 1 90); do
  n=$(rosnode list 2>/dev/null | wc -l)
  if [ "$n" -ge 20 ]; then ok=1; break; fi
  sleep 0.5
done
if [ "$ok" != 1 ]; then echo "FAIL: nodes=$n <20"; kill $LPID 2>/dev/null; exit 1; fi
echo "READY nodes=$n"

# 挂监测器（被动、记录存证）
rm -f /tmp/fm_test.jsonl /tmp/fm_test.txt
python3 tools/fleet_monitor.py --noline --rate 1 --record /tmp/fm_test.jsonl >/tmp/fm_test.txt 2>&1 &
MPID=$!
sleep 2

# 开赛
rosservice call /zx2026/start || true

# 等 DONE（<=420s）
bash tools/monitor_wait_state.sh DONE 420
RC=$?

sleep 2
kill $MPID 2>/dev/null

echo "=== monitor dashboard tail ==="
tail -c 4500 /tmp/fm_test.txt
echo "=== record lines ==="
wc -l /tmp/fm_test.jsonl
echo "=== record last frame ==="
tail -1 /tmp/fm_test.jsonl

# 拆栈（同 run_verify.sh 结尾）
kill $LPID 2>/dev/null
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "E2E_RC=$RC"
exit $RC
