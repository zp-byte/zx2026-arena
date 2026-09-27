#!/bin/zsh
# =============================================================================
# boot_all.zsh — 上电一键基础设施链（2026-09-27 固化）
# 范围：对时→清场→roscore→mavros(仅 IMU 数据链)→faster_lio→ekf→cloud_adapter
#       →D435→color_detector
#       【不含飞行控制栈（px4ctrl/vel_bridge/real_fleet）——无解锁/OFFBOARD/
#         任何指令通路，FCU 纯被动读。mavros 在这里只是 LIO/EKF 的 IMU 数据源
#         （faster_lio mid360.yaml imu_topic=/mavros/imu/data 定谳），FCU 须上电】
# 用法：
#   笔记本侧（推荐）：bash boot_zx.sh     ← 自动烘 epoch、scp、执行、拉看门狗
#   机上本地：zsh ~/boot/boot_all.zsh [epoch] ← 无参用装订 epoch（断电重启钟回 1970）
# 配方来源：lio_bg/cloud_bg/boot_color/rs_restart 四脚本定谳版合体。
#   教训：epoch 必须写死进脚本（引号层吃 $var）；set_xu busy 须杀净等 5s；
#   USB 瞬断后 detector 空壳须与 rs 同拉。
# =============================================================================
BOOT_EPOCH_BAKED=__BOOT_EPOCH__

OK=0; FAIL=0
ok()  { echo "[OK]   $1"; OK=$((OK+1)); }
bad() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

wait_stream() { # topic budget_s label——在列+有流双判据
  local t=$1 lim=$2 label=$3 i=0
  while [ $i -lt $lim ]; do
    if timeout 3 rostopic list 2>/dev/null | grep -qx "$t"; then
      if timeout 6 rostopic hz "$t" -w 5 2>&1 | grep -q "average rate"; then
        echo "[OK]   $label ($t)"; OK=$((OK+1)); return 0
      fi
    fi
    sleep 1; i=$((i+1))
  done
  echo "[FAIL] $label ($t) ${lim}s 未上线/无流"; FAIL=$((FAIL+1)); return 1
}

# ---- 0. 对时 ----
NOW=$(date +%s)
if [ -n "$1" ]; then E=$1
elif [ "$NOW" -lt 1780000000 ]; then E=$BOOT_EPOCH_BAKED
else E=0; fi
if [ "$E" != "0" ]; then
  echo nv | sudo -S date -s @$E >/dev/null 2>&1
fi
echo "== 机钟 $(date '+%F %T')（epoch $(date +%s)）"

# ---- 1. 清场（含看门狗；脚本名无 ros 串防 pkill 自杀） ----
pkill -f "d435_watchdo[g]" 2>/dev/null
pkill -f "roslaunch" 2>/dev/null; pkill -f "rosmaste[r]" 2>/dev/null; pkill -f "rosou[t]" 2>/dev/null
pkill -f "run_mapping_[o]nline" 2>/dev/null; pkill -f "livox_ros_driver2_[n]ode" 2>/dev/null
pkill -f "cloud_[a]dapter.py" 2>/dev/null
pkill -f "rs_camer[a]" 2>/dev/null; pkill -f "realsense2_camera_manage[r]" 2>/dev/null
pkill -f "color_detector_nod[e]" 2>/dev/null
sleep 3
pgrep -f "rosmaste[r]" >/dev/null && { pkill -9 -f "rosmaste[r]"; sleep 1; }

# ---- 2. 环境配方（lio_bg.zsh 定谳版） ----
export ROS_MASTER_URI=http://localhost:11311
source $HOME/Diff-planner/devel/setup.zsh
export ROS_PACKAGE_PATH=$HOME/zx2026_ws/src:$ROS_PACKAGE_PATH
export PYTHONPATH=$HOME/zx2026_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$HOME/zx2026_ws/devel/lib:$HOME/Diff-planner/devel/lib:$LD_LIBRARY_PATH
export PATH=$HOME/zx2026_ws/devel/lib:$PATH
export BD_LIST=47MDM630020460
export LIDAR_IP=192.168.2.88

# ---- 3. roscore ----
if ! timeout 4 rostopic list >/dev/null 2>&1; then
  nohup roscore > /tmp/roscore.log 2>&1 &
  i=0
  while [ $i -lt 20 ]; do
    timeout 3 rostopic list >/dev/null 2>&1 && break
    sleep 1; i=$((i+1))
  done
fi
timeout 4 rostopic list >/dev/null 2>&1 && ok "roscore" || { bad "roscore 未起"; exit 1; }

# ---- 3.5 mavros（仅 IMU 数据链，非控制链；FCU 须上电） ----
cd ~/Diff-planner
nohup roslaunch mavros px4.launch > /tmp/mavros.log 2>&1 &
wait_stream /mavros/imu/data 30 "mavros IMU（LIO/EKF 数据源）"

# ---- 4. faster_lio ----
nohup roslaunch faster_lio mapping_mid360.launch > /tmp/faster_lio.log 2>&1 &
wait_stream /livox/lidar 25 "MID360 原始点云"
wait_stream /laserMapping/odometry 30 "faster_lio 里程计"

# ---- 4.5 ekf（订 mavros IMU + LIO 里程计 → /ekf/ekf_odom；包真名=ekf，源码目录叫 ekf_pose） ----
nohup roslaunch ekf ekf_lidar.launch > /tmp/ekf.log 2>&1 &
wait_stream /ekf/ekf_odom 20 "EKF 融合里程计"

# ---- 5. cloud_adapter（源=/laserMapping 前缀定谳值） ----
nohup rosrun arena_bridges cloud_adapter.py --id 0 --src /laserMapping/cloud_registered \
  --odom /ekf/ekf_odom --yaw0-deg 0 --ex0 1.0 --ny0 23.0 > /tmp/cloud_adapter.log 2>&1 &
sleep 4
wait_stream /drone_0/cloud 15 "drone_0/cloud（导航消费云）"

# ---- 6. D435（降流配方；杀净等 5s 防 set_xu busy） ----
pkill -f "rs_camer[a]" 2>/dev/null; pkill -f "realsense2_camera_manage[r]" 2>/dev/null
sleep 5
cd ~/Diff-planner
nohup roslaunch realsense2_camera rs_camera.launch enable_infra1:=false \
  enable_infra2:=false enable_sync:=false depth_fps:=15 color_fps:=15 > /tmp/rs_camera.log 2>&1 &
wait_stream /camera/color/image_raw 30 "D435 color 流"
wait_stream /camera/depth/image_rect_raw 20 "D435 depth 流"
grep -q "overflow" /tmp/rs_camera.log 2>/dev/null && echo "[WARN] rs_camera.log 有 overflow"

# ---- 7. color_detector（与 rs 同拉——空壳教训） ----
pkill -f "color_detector_nod[e]" 2>/dev/null; sleep 1
nohup rosrun arena_sensor color_detector_node.py _drone_id:=0 _image_topic:=/camera/color/image_raw > /tmp/cd.log 2>&1 &
sleep 4
wait_stream /drone_0/detected/color 15 "颜色检出话题"

# ---- 8. 汇总 ----
echo "== 定位样点 /ekf/ekf_odom（LIO 原点=起飞点，应近 0；ENU 对齐须起任务链 odom_tf）=="
timeout 4 rostopic echo -n1 /ekf/ekf_odom/pose/pose/position 2>/dev/null | grep -E "^[xyz]:" || echo "(样点超时)"
echo "== 汇总 OK=$OK FAIL=$FAIL =="
echo "== 本脚本只起传感器链，不含飞行栈；任务链另起 real_fleet.launch =="
exit $FAIL
