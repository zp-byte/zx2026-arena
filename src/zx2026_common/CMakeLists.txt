cmake_minimum_required(VERSION 3.0.2)
project(zx2026_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Mission.msg
  Score.msg
  TaskUpdate.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
)
