#!/usr/bin/env bash
# 可视化一键启动：全栈 + RViz（WSLg 窗口显示在 Windows 桌面）
# 用法: bash /home/ubuntu/zx2026_arena_ws/tools/viz_run.sh
# 之后: 另开终端  rosservice call /zx2026/start  开始比赛
#       另开终端  rostopic echo /zx2026/score_summary  看总分
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0
# WSLg 下 RViz 用软件 GL，保证窗口正常渲染
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe

find /home/ubuntu/zx2026_arena_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 1

nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_viz_stack.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 90); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5; done
echo "全栈节点数: $(rosnode list 2>/dev/null | wc -l)"

exec rviz -d /home/ubuntu/zx2026_arena_ws/src/arena_world/config/zx2026.rviz
