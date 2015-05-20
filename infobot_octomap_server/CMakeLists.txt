cmake_minimum_required(VERSION 2.8.3)
project(infobot_octomap_server)

find_package(catkin REQUIRED roscpp octomap_server octomap_msgs nav_msgs map_server)
find_package(octomap REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  CATKIN_DEPENDS roscpp octomap_server octomap_msgs nav_msgs map_server
  DEPENDS octomap
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(octomap_from_map_static_server src/octomap_from_map_static_server.cpp)
add_dependencies(octomap_from_map_static_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_from_map_static_server ${catkin_LIBRARIES} yaml-cpp)

## Install executables and/or libraries
install(TARGETS octomap_from_map_static_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
