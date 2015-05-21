cmake_minimum_required(VERSION 2.8.3)
project(infobot_octomap_store)

find_package(catkin REQUIRED
  roscpp
  octomap_server
  octomap
  infobot_map_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  CATKIN_DEPENDS roscpp octomap_server infobot_map_msgs
  # DEPENDS
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(octomap_store src/octomap_store.cpp)
add_dependencies(octomap_store ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_store
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Install executables and/or libraries
install(TARGETS octomap_store
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
