cmake_minimum_required(VERSION 3.0.2)
project(fruit_counting)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
)

add_message_files(
  FILES
  SyncedPcOdomImg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  nav_msgs
)
