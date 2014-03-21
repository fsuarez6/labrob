cmake_minimum_required(VERSION 2.8.3)
project(labrob_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  joy
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES labrob_control
#  CATKIN_DEPENDS controller_manager geometry_msgs joy rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
