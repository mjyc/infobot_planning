cmake_minimum_required(VERSION 2.8.3)
project(infobot_map_store)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

generate_messages(DEPENDENCIES)

catkin_package()
