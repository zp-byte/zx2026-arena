cmake_minimum_required(VERSION 3.0.2)
project(arena_world_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs tf2_ros gazebo_ros
)
find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
  tf2_ros gazebo_ros)

# 无人机速度控制 Gazebo 插件
add_library(drone_vel_plugin src/drone_vel_plugin.cpp)
target_include_directories(drone_vel_plugin PUBLIC
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(drone_vel_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS drone_vel_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
