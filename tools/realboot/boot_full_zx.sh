#!/bin/bash
# =============================================================================
# boot_full_zx.sh — 断电重启全链一键（笔记本 WSL 侧跑；2026-09-27 固化）
# 用法：bash tools/realboot/boot_full_zx.sh          （地面干转档，默认）
#       bash tools/realboot/boot_full_zx.sh --full   （加挂 px4ctrl 无桨挂起档）
# 分层：[1] 传感器链=boot_zx.sh（烘epoch+9检+看门狗，boot_all 红线不含飞行栈）
#       [2-5] odom_tf → 大脑层5节点 → GCS agent；[--full] px4ctrl
# 配方来源：2026-09-27 逐条实弹验证序列原样搬入（brain_run/px4_run/gcs 干跑全绿）
# 纪律：环境串整体抄勿拆——PYTHONPATH 必须带 :$PYTHONPATH 追加继承，否则洗掉 rospy；
#       清场独立 ssh+全括号模式防 pkill 自杀雷；px4ctrl 活体证据看 rosout 非 stdout
# 前提：机上 nv 免密 ssh（192.168.31.76）；DHCP 变动先改 D 或 provision --server
# =============================================================================
set -e
D=nv@192.168.31.76
FULL=0
[ "${1:-}" = "--full" ] && FULL=1
# 机上 ROS 环境配方（px4_run.sh 验证过的 "$E " 前缀式：笔记本侧单层展开，$HOME 留给远端）
E='source ~/Diff-planner/devel/setup.zsh >/dev/null 2>&1; export PYTHONPATH=$HOME/zx2026_ws/devel/lib/python3/dist-packages:$PYTHONPATH;'

echo "===== [1/6] 传感器链一键（boot_zx：烘 epoch + 9 检 + 看门狗）====="
bash "$(cd "$(dirname "$0")" && pwd)/boot_zx.sh"

echo "===== [2/6] 应用层清场（独立 ssh，全括号模式防自杀雷）====="
ssh $D 'pkill -f "nav_node.p[y]" 2>/dev/null; pkill -f "mission_executor_node.p[y]" 2>/dev/null; pkill -f "mission_loader.p[y]" 2>/dev/null; pkill -f "type_match_node.p[y]" 2>/dev/null; pkill -f "vel_bridge.p[y]" 2>/dev/null; pkill -f "odom_tf.p[y]" 2>/dev/null; pkill -f "gcs_agent.p[y]" 2>/dev/null; pkill -f "px4ctrl_nod[e]" 2>/dev/null; sleep 1; echo CLEANED'

echo "===== [3/6] odom_tf（ENU 对齐，ex0=1.0 ny0=23.0）====="
ssh $D "$E nohup python3 ~/zx2026_arena_ws/tools/gcs/odom_tf.py --odom /ekf/ekf_odom --out /drone_0/odom --yaw0-deg 0 --ex0 1.0 --ny0 23.0 > /tmp/odom_tf.log 2>&1 & sleep 3; tail -2 /tmp/odom_tf.log"

echo "===== [4/6] 大脑层五节点（PYTHONPATH 继承配方整体抄）====="
ssh $D "$E nohup python3 ~/zx2026_ws/src/arena_bridges/scripts/mission_loader.py --id 0 --yaml ~/zx2026_arena_ws/tools/gcs/missions.yaml > /tmp/ml.log 2>&1 & nohup python3 ~/zx2026_ws/src/arena_mission/scripts/type_match_node.py > /tmp/tm.log 2>&1 & nohup python3 ~/zx2026_ws/src/arena_nav/scripts/nav_node.py > /tmp/nav.log 2>&1 & nohup python3 ~/zx2026_ws/src/arena_mission/scripts/mission_executor_node.py > /tmp/exe.log 2>&1 & nohup python3 ~/zx2026_ws/src/arena_bridges/scripts/vel_bridge.py --id 0 --odom /ekf/ekf_odom --yaw0-deg 0 --max-vel 1.2 --out /setpoints_cmd > /tmp/vb.log 2>&1 & sleep 8; for f in ml tm nav exe vb; do echo \"-- \$f:\"; tail -1 /tmp/\$f.log; done"

echo "===== [5/6] GCS agent（上行+fleet 下行入池）====="
ssh $D "$E nohup python3 ~/zx2026_arena_ws/tools/gcs/gcs_agent.py --profile ~/zx2026_arena_ws/tools/gcs/profile_real_lio.yaml --fleet-bridge > /tmp/gcs_agent.log 2>&1 & sleep 4; tail -3 /tmp/gcs_agent.log"

if [ "$FULL" = "1" ]; then
  echo "===== [6/6] px4ctrl（--full 无桨挂起档：RC 不开则驻 MANUAL_CTRL 等待，安全）====="
  ssh $D "$E nohup roslaunch px4ctrl run_ctrl_lio.launch > /tmp/px4ctrl.log 2>&1 & sleep 7; echo PX4CTRL_LAUNCHED"
fi

echo "===== 自检（活体证据以 rosout/rostopic 为准，stdout 缓冲不可信）====="
N=$(ssh $D 'source ~/Diff-planner/devel/setup.zsh >/dev/null 2>&1; rosnode list 2>/dev/null | grep -cE "nav_node|mission_executor|mission_loader|type_match_node|vel_bridge"') || N=0
[ "$N" -ge 5 ] && echo "BRAIN_NODES OK($N/5)" || echo "BRAIN_NODES FAIL($N/5)"
O=$(ssh $D "$E timeout 4 rostopic echo -n1 /drone_0/odom 2>/dev/null | head -1") || O=""
[ -n "$O" ] && echo "ODOM_TF OK($O)" || echo "ODOM_TF FAIL"
A=$(ssh $D 'tail -5 /tmp/gcs_agent.log 2>/dev/null | grep -cE "connect|report|down"') || A=0
[ "$A" -ge 1 ] && echo "GCS_AGENT OK" || echo "GCS_AGENT FAIL(查 /tmp/gcs_agent.log)"
if [ "$FULL" = "1" ]; then
  C=$(ssh $D 'source ~/Diff-planner/devel/setup.zsh >/dev/null 2>&1; rostopic info /setpoints_cmd 2>/dev/null' | grep -c px4ctrl) || C=0
  [ "$C" -ge 1 ] && echo "CMD_LOOP OK(vel_bridge→px4ctrl 闭环)" || echo "CMD_LOOP FAIL(px4ctrl 未收到信方在列)"
  R=$(ssh $D 'grep -c "px4ct[r]l" ~/.ros/log/latest/rosout.log 2>/dev/null') || R=0
  [ "$R" -ge 1 ] && echo "PX4CTRL ALIVE(rosout $R 行，Waiting for RC=预期)" || echo "PX4CTRL FAIL(rosout 无发言)"
fi

echo "----- GCS hub（笔记本 Windows 侧手动起，复制下行）-----"
echo 'py -3 //wsl.localhost/Ubuntu-20.04/home/ubuntu/zx2026_arena_ws/tools/gcs/gcs_hub.py --view log --ids 0 --port 9870 --fleet-hz 5 --bat-min 30 --logdir C:\Users\24882\AppData\Local\Temp\zx_gcs'
echo "=================================================="
if [ "$FULL" = "1" ]; then echo "BOOT_FULL_RC=0 (full: 传感器链+大脑层+GCS+px4ctrl 挂起)"; else echo "BOOT_FULL_RC=0 (ground: 传感器链+大脑层+GCS，无飞行栈)"; fi
