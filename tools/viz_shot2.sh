#!/usr/bin/env bash
# 启动全栈 + RViz（软件 GL），多策略截图：窗口抓取 / xwd 全屏
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe

find /home/ubuntu/zx2026_arena_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_viz_stack.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 90); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5; done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"
rosservice call /zx2026/start >/dev/null 2>&1

nohup rviz -d /home/ubuntu/zx2026_arena_ws/src/arena_world/config/zx2026.rviz >/tmp/zx2026_rviz.log 2>&1 &
RV=$!
sleep 14
echo "rviz alive: $(kill -0 $RV 2>/dev/null && echo yes || echo no)"

# 策略1：按名称找主窗口
WID=$(DISPLAY=:0 xdotool search --name "rviz" 2>/dev/null | tail -1)
echo "window(name rviz): $WID"
if [ -n "$WID" ]; then
  DISPLAY=:0 import -window "$WID" /tmp/shot_win.png 2>/dev/null && identify /tmp/shot_win.png
fi

# 策略2：xwd 全屏根窗口
DISPLAY=:0 xwd -root -silent 2>/dev/null | convert xwd:- /tmp/shot_root.png 2>/dev/null \
  && identify /tmp/shot_root.png

# 策略3：截取 3D 视口区域（RViz 主窗口中心 1200x700）
if [ -n "$WID" ]; then
  eval $(DISPLAY=:0 xdotool getwindowgeometry --shell "$WID")
  if [ -n "$WIDTH" ] && [ "$WIDTH" -gt 200 ]; then
    X=$((X + WIDTH/2 - 600)); [ $X -lt 0 ] && X=0
    DISPLAY=:0 import -window root -crop 1200x700+$X+100 /tmp/shot_crop.png 2>/dev/null \
      && identify /tmp/shot_crop.png
  fi
fi

kill $RV 2>/dev/null; sleep 1
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "roscore.*11411" 2>/dev/null
echo "== SHOT2 DONE =="
