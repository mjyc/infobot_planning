cmake_minimum_required(VERSION 2.8.3)
project(infobot_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  ProbabilityMapMetaData.msg
  ProbabilityGrid.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  # Map
  ListMapFiles.srv
  PublishMap.srv
  # Octomap
  GetMap.srv
  ListOctomapFiles.srv
  PublishOctomap.srv
  GetOctomap.srv
  # ProbabilityMap
  ListProbabilityMapFiles.srv
  PublishProbabilityMap.srv
  GetProbabilityMap.srv
  AddDisjointProbabilityMap.srv
  MultiplyJointProbabilityMap.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs octomap_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs std_msgs nav_msgs octomap_msgs)
