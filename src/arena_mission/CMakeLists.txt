cmake_minimum_required(VERSION 3.0.2)
project(arena_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs sensor_msgs std_srvs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs std_srvs)
