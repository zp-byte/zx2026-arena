#!/bin/bash
# env.sh — 交互终端 ROS 环境（配合 stack_up.sh 后台栈用）
# 坑源（2026-09-06 法证）：ops 的 trigger 子进程需要 ROS 环境继承，
# 裸终端跑 ops → "Unable to communicate with master!"（master 在 11411 非 11311）。
# 用法:  source tools/gcs/env.sh   然后正常跑 gcs_ops.py / gcs_panel.py
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/devel/setup.bash"
echo "[env.sh] ROS_MASTER_URI=$ROS_MASTER_URI  nodes=$(rosnode list 2>/dev/null | wc -l)"
