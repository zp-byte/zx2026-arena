#!/bin/bash
# 干净关闭所有仿真进程（roslaunch SIGINT -> 残留 SIGKILL）
# 用脚本文件而非 wsl bash -c 内联，避免 pkill -f 模式自匹配杀掉 shell

# 1) roslaunch 父进程 SIGINT -> 触发它清理所有子节点
RL=$(pgrep -f "roslaunch arena_world")
if [ -n "$RL" ]; then
  kill -INT $RL 2>/dev/null
  echo "SIGINT roslaunch (pid $RL), 等待清理..."
  sleep 3
fi

# 2) SIGKILL 残留：gzserver/gzclient/rviz/rosmaster/rosout + 所有 arena 节点
#    按 PID 杀，不用 -f 模式，避免误伤
PIDS=$(pgrep -f "gzserver|gzclient|rviz|rosmaster|rosout|scene_marker_node|fleet_node|tag_detector_node|nav_node|mission_executor_node|task_generator_node|type_match_node|drop_simulator_node|stage_controller_node|scorekeeper_node" 2>/dev/null)
if [ -n "$PIDS" ]; then
  echo "SIGKILL 残留: $PIDS"
  kill -9 $PIDS 2>/dev/null
  sleep 1
fi

# 3) 最终检查
echo "=== 最终残留检查 ==="
LEFT=$(pgrep -af "gzserver|gzclient|rviz|rosmaster|rosout|arena_world|arena_nav|arena_fleet|arena_sensor|arena_mission|arena_score|nav_node|mission_executor|fleet_node|stage_controller|scorekeeper|task_generator|type_match|drop_simulator|tag_detector|scene_marker|lidar_node" 2>/dev/null | grep -v "$0" | grep -v pgrep)
if [ -n "$LEFT" ]; then
  echo "仍有残留:"
  echo "$LEFT"
else
  echo "全部已关闭"
fi
