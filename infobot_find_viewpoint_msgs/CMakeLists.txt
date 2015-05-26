cmake_minimum_required(VERSION 2.8.3)
project(infobot_find_viewpoint_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  octomap_msgs
  infobot_map_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  ComputeVisibilityValue.srv
  ComputeVisibilityValues.srv
  GetProbabilityOctomapSurface.srv
  GetProbabilityOctomapHeight.srv
)

add_action_files(
  DIRECTORY
    action
  FILES
    FindViewpoint.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs octomap_msgs infobot_map_msgs)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs geometry_msgs sensor_msgs octomap_msgs infobot_map_msgs
    # DEPENDS
)
