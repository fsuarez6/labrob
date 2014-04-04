cmake_minimum_required(VERSION 2.8.3)
project(labrob_description)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  urdf
  xacro
  turtlebot_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES labrob_description
#  CATKIN_DEPENDS robot_state_publisher urdf xacro
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
