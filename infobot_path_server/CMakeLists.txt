cmake_minimum_required(VERSION 2.8.3)
project(infobot_path_server)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  tf
  std_msgs
  geometry_msgs
  nav_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  GetPath.srv
)

generate_messages(DEPENDENCIES tf std_msgs geometry_msgs nav_msgs)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS message_runtime tf std_msgs geometry_msgs nav_msgs
    # DEPENDS
)
