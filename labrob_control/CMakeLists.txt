cmake_minimum_required(VERSION 2.8.3)
project(labrob_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  geometry_msgs
  joy
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
