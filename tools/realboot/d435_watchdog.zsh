#!/bin/zsh
# =============================================================================
# d435_watchdog.zsh — D435 颜色链看门狗（2026-09-27）
# 规则：/camera/color/image_raw 无流连续 2 检（~11s）→ 重拉 rs+detector（必须同拉——
#   USB 瞬断 detector 空壳教训）；图像有流但检出流断 2 检 → 只重拉 detector。
# 日志：/tmp/d435_watchdog.log
# 启动：nohup zsh ~/boot/d435_watchdog.zsh > /dev/null 2>&1 &
# =============================================================================
export ROS_MASTER_URI=http://localhost:11311
source $HOME/Diff-planner/devel/setup.zsh
export ROS_PACKAGE_PATH=$HOME/zx2026_ws/src:$ROS_PACKAGE_PATH
export PYTHONPATH=$HOME/zx2026_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$HOME/zx2026_ws/devel/lib:$HOME/Diff-planner/devel/lib:$LD_LIBRARY_PATH
export PATH=$HOME/zx2026_ws/devel/lib:$PATH

IMG_DEAD=0; DET_DEAD=0
echo "$(date '+%F %T') watchdog start" >> /tmp/d435_watchdog.log
while true; do
  if timeout 6 rostopic hz /camera/color/image_raw -w 3 2>&1 | grep -q "average rate"; then
    IMG_DEAD=0
  else
    IMG_DEAD=$((IMG_DEAD+1))
    echo "$(date '+%F %T') img dead x$IMG_DEAD" >> /tmp/d435_watchdog.log
  fi
  if timeout 6 rostopic hz /drone_0/detected/color -w 3 2>&1 | grep -q "average rate"; then
    DET_DEAD=0
  else
    DET_DEAD=$((DET_DEAD+1))
    echo "$(date '+%F %T') det dead x$DET_DEAD" >> /tmp/d435_watchdog.log
  fi
  if [ $IMG_DEAD -ge 2 ]; then
    echo "$(date '+%F %T') RESTART rs+detector" >> /tmp/d435_watchdog.log
    pkill -f "rs_camer[a]" 2>/dev/null; pkill -f "realsense2_camera_manage[r]" 2>/dev/null
    pkill -f "color_detector_nod[e]" 2>/dev/null
    sleep 5
    cd ~/Diff-planner
    nohup roslaunch realsense2_camera rs_camera.launch enable_infra1:=false \
      enable_infra2:=false enable_sync:=false depth_fps:=15 color_fps:=15 > /tmp/rs_camera.log 2>&1 &
    sleep 14
    nohup rosrun arena_sensor color_detector_node.py _drone_id:=0 _image_topic:=/camera/color/image_raw > /tmp/cd.log 2>&1 &
    IMG_DEAD=0; DET_DEAD=0
  elif [ $DET_DEAD -ge 2 ]; then
    echo "$(date '+%F %T') RESTART detector only" >> /tmp/d435_watchdog.log
    pkill -f "color_detector_nod[e]" 2>/dev/null
    sleep 2
    nohup rosrun arena_sensor color_detector_node.py _drone_id:=0 _image_topic:=/camera/color/image_raw > /tmp/cd.log 2>&1 &
    DET_DEAD=0
  fi
  sleep 5
done
