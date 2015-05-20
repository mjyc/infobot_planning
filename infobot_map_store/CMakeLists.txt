cmake_minimum_required(VERSION 2.8.3)
project(infobot_map_store)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  nav_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  ListMapFiles.srv
  PublishMap.srv
  GetMap.srv
)

generate_messages(DEPENDENCIES std_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs)
