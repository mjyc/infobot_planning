cmake_minimum_required(VERSION 2.8.3)
project(infobot_topo_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs geometry_msgs message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  TopologicalMap.msg
  TopologicalPlace.msg
  TopologicalView.msg
  MetricMapTransform.msg)


add_service_files(
  DIRECTORY srv
  FILES
  ApplyRectangleFilter.srv
  GetTopologicalMap.srv
  ListTopologicalMapFiles.srv
  ProcessMetricMap.srv
  ProcessCurrentMetricMap.srv
  PublishTopologicalMap.srv
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_runtime)
