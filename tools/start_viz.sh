#!/usr/bin/env bash
# 单独启动 RViz 连现有栈（不重启全栈）；比赛需另调 /zx2026/start
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe

pkill -x rviz 2>/dev/null
sleep 1
setsid nohup rviz -d /home/ubuntu/zx2026_arena_ws/src/arena_world/config/zx2026.rviz >/tmp/zx2026_rviz.log 2>&1 < /dev/null &
sleep 6
if pgrep -x rviz >/dev/null; then
  echo "RViz up (pid $(pgrep -x rviz))"
else
  echo "RViz FAILED"
  tail -n 10 /tmp/zx2026_rviz.log
fi
