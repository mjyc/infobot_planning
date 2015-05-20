cmake_minimum_required(VERSION 2.8.3)
project(infobot_pmap_store)

find_package(catkin REQUIRED COMPONENTS
  rospy
  infobot_mapping_msgs
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  ListProbabilityMapFiles.srv
  PublishProbabilityMap.srv
  GetProbabilityMap.srv
  AddDisjointProbabilityMap.srv
  MultiplyJointProbabilityMap.srv
)

generate_messages(DEPENDENCIES std_msgs infobot_mapping_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs infobot_mapping_msgs)
