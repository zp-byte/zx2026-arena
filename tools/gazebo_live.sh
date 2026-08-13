#!/usr/bin/env bash
# 启动 Gazebo 仿真（gzserver 物理后端）+ gzclient（3D GUI）+ RViz，保持运行供用户查看。
# 关键：OGRE_RTT_MODE=Copy 避免 gzclient 的 OGRE FBO 渲染线程与 gzserver 在 WSLg 软件 GL
#       下死锁（表现为 gzclient 一连上 /clock 就停摆）。已实测稳定。
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export OGRE_RTT_MODE=Copy
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

# 9P/UNC 编辑会清掉 .py 可执行位 → nav_node 等起不来；启动前统一恢复
find /home/ubuntu/zx2026_arena_ws/src -name '*.py' -exec chmod +x {} + 2>/dev/null

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 2
# setsid 脱离调用进程组，保证工具调用结束后仿真继续运行
setsid nohup roslaunch arena_world_gazebo gazebo.launch viewer:=true \
  >/tmp/zx2026_gazebo_live.log 2>&1 < /dev/null &
disown 2>/dev/null || true

# 等 gzserver 起来后再拉 gzclient（连早了易死锁）
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 120); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done
# gzclient 连已运行的 gzserver（OGRE_RTT_MODE=Copy 已在环境里，防死锁）
setsid nohup gzclient >/tmp/zx2026_gzclient.log 2>&1 < /dev/null &
disown 2>/dev/null || true
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"

# 等 P2_WAIT 后 start（若已在跑则不重复）
ST=$(timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
echo "state: $ST"
if [ "$ST" = '"P2_WAIT"' ]; then rosservice call /zx2026/start >/dev/null 2>&1; echo "started"; fi

echo "--- live 状态 ---"
echo "gzserver: $(pgrep -f 'gzserver /home' >/dev/null && echo UP || echo DOWN) | gzclient: $(pgrep -f 'gzclient' >/dev/null && echo UP || echo DOWN) | rviz: $(pgrep -f rviz >/dev/null && echo UP || echo DOWN)"
echo "clock: $(timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -o 'secs: [0-9]*' | head -1)"
echo "markers: $(timeout 4 rostopic echo -n1 /zx2026/markers 2>/dev/null | grep -c 'ns: forest') 个森林 marker"
echo "odom0: $(timeout 3 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -E 'x:|y:' | head -2 | tr '\n' ' ')"
echo "仿真保持运行：ROS_MASTER_URI=http://127.0.0.1:11411，gzclient + RViz 窗口在 Windows 桌面可见"
echo "== LIVE =="

