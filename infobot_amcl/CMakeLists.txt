cmake_minimum_required(VERSION 2.8.3)
project(infobot_amcl)

find_package(catkin REQUIRED roscpp geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_initialpose src/pub_initialpose.cpp)
add_dependencies(pub_initialpose ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_initialpose
  ${catkin_LIBRARIES}
)
