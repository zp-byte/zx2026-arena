cmake_minimum_required(VERSION 3.0.2)
project(arena_world)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs sensor_msgs std_srvs visualization_msgs rosgraph_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs
  std_srvs visualization_msgs rosgraph_msgs)
