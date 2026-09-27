cmake_minimum_required(VERSION 3.0.2)
project(arena_bridges)

# 纯脚本 shim 包（无编译产物；devel 空间 roslaunch 按 pkg/type 发现 scripts/）
find_package(catkin REQUIRED)
catkin_package()
