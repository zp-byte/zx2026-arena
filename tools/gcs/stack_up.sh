#!/bin/bash
# stack_up.sh — GCS+仿真 一键起栈（清场 → sim → agent → hub → 验证后退出）
#
# 用法（在交互终端里跑，栈宿主本终端会话，关终端即停）：
#   nohup bash tools/gcs/stack_up.sh > /tmp/stack_up.log 2>&1 & disown
#   tail -f /tmp/stack_up.log        # 等待 "STACK READY"
#
# 设计要点（2026-09-06 实战法证）：
#   * 不起面板——面板留给交互终端自己带 ROS 环境跑（ops 子进程继承环境，
#     否则 START 报 "Unable to communicate with master"）。
#   * 验证后脚本即退出，进程脱离脚本存活（宿主=终端会话，而非本脚本）。
#   * 铁律：读 gcs_status.json 诊断前先 stat mtime——hub 死后文件冻结成化石。
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"

echo "=== CLEAN ==="
for pat in "roslaunch" "rvizi[c]" "gzserve[r]" "gzclien[t]" "rosmaste[r]" \
           "gcs_age[n]" "gcs_hu[b]" "executor_" "nav_nod[e]" \
           "stage_controlle[r]" "world_nod[e]" "scorekeepe[r]" "fleet_nod[e]" \
           "comm_mode[l]" "tag_detecto[r]" "collision_monito[r]"; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2
pkill -9 -f "roslaunc[h]" 2>/dev/null
pkill -9 -f "rosmaste[r]" 2>/dev/null
sleep 1
echo "CLEAN: master=$(pgrep -c rosmaste[r] || true)"

echo "=== SIM ==="
roslaunch arena_world zx2026_all.launch > /tmp/sim_stack_up.log 2>&1 &
for i in $(seq 1 120); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 120); do
  N=$(rosnode list 2>/dev/null | wc -l)
  S=$(rosnode list 2>/dev/null | grep -c stage_controller)
  [ "$N" -ge 35 ] && [ "$S" -ge 1 ] && break
  sleep 0.5
done
echo "SIM UP: nodes=$(rosnode list 2>/dev/null | wc -l)"
timeout 5 rosservice info /zx2026/start >/dev/null 2>&1 \
  && echo "SERVICE /zx2026/start OK" || echo "SERVICE /zx2026/start MISSING"

echo "=== AGENT ==="
python3 "$ROOT/tools/gcs/gcs_agent.py" \
  --profile "$ROOT/tools/gcs/profile_sim.yaml" > /tmp/gcs_agent_stack.log 2>&1 &
sleep 4

echo "=== HUB ==="
python3 "$ROOT/tools/gcs/gcs_hub.py" --ids 0,1,2,3,4,5 --port 9870 \
  > /tmp/gcs_hub_stack.log 2>&1 &
sleep 8

echo "=== VERIFY ==="
python3 - <<'PY'
import json, os
p = os.path.expanduser("~/zx2026_arena_ws/run_logs/gcs_status.json")
d = json.load(open(p))
pat = "".join("X" if (d["drones"][k] or {}).get("phase") else "."
              for k in sorted(d["drones"]))
print("pattern:", pat, "stage:", d.get("stage"))
print("grids:", sorted((d.get("grids") or {}).keys()) or "[] (起飞后出现=预期)")
PY
echo "STACK READY（宿主=本终端会话, 关终端即停）"
