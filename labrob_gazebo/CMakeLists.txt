cmake_minimum_required(VERSION 2.8.3)
project(labrob_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  labrob_description
  turtlebot_bringup
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES labrob_gazebo
#  CATKIN_DEPENDS gazebo_ros labrob_description
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
