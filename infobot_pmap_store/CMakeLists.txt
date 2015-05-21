cmake_minimum_required(VERSION 2.8.3)
project(infobot_pmap_store)

find_package(catkin REQUIRED COMPONENTS
  rospy
  infobot_map_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy infobot_map_msgs
)
