cmake_minimum_required(VERSION 2.8.3)
project(labrob_worlds)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  labrob_msgs
  nav_msgs
)


catkin_package(
CATKIN_DEPENDS 
  roscpp
  gazebo_ros
  labrob_msgs
  nav_msgs
)

catkin_python_setup()

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(coins_manager_plugin src/coins_manager_plugin.cpp)
target_link_libraries(coins_manager_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
