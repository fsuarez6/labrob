cmake_minimum_required(VERSION 2.8.3)
project(labrob_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation nav_msgs)

add_service_files(
    FILES
    GetMapPercentage.srv
)

generate_messages(DEPENDENCIES nav_msgs std_msgs)

catkin_package(
CATKIN_DEPENDS 
  message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

