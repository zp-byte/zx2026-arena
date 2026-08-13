#!/usr/bin/env bash
# 可视化冒烟测试：启动全栈 → 检查 /tf 机体帧 → 启动 RViz → 截图
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0

find /home/ubuntu/zx2026_arena_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_viz_stack.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 90); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5; done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"

TF_COUNT=$(timeout 6 rostopic echo /tf 2>/dev/null | grep -c child_frame_id)
echo "tf drone frames seen: $TF_COUNT"

rosservice call /zx2026/start >/dev/null 2>&1
echo "start called"

nohup rviz -d /home/ubuntu/zx2026_arena_ws/src/arena_world/config/zx2026.rviz >/tmp/zx2026_rviz.log 2>&1 &
RV=$!
sleep 10
if kill -0 $RV 2>/dev/null; then echo "rviz ALIVE pid=$RV"; else echo "rviz DIED"; fi
import -display :0 -window root /tmp/zx2026_rviz.png 2>/tmp/zx2026_shot.log \
  && echo "screenshot OK $(du -h /tmp/zx2026_rviz.png | cut -f1)" \
  || cat /tmp/zx2026_shot.log

kill $RV 2>/dev/null; sleep 1
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "roscore.*11411" 2>/dev/null
echo "== VIZTEST DONE =="
