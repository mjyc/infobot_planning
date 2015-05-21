cmake_minimum_required(VERSION 2.8.3)
project(infobot_find_viewpoint)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  octomap_msgs
  infobot_map_msgs
  infobot_topo_msgs
  visualization_msgs
)
find_package(octomap REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS roscpp geometry_msgs octomap_msgs infobot_map_msgs infobot_topo_msgs visualization_msgs
    DEPENDS octomap
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(pmap_to_octomap src/pmap_to_octomap_node.cpp)
add_dependencies(pmap_to_octomap ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmap_to_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(visibility_reasoner src/visibility_reasoner_node.cpp)
add_dependencies(visibility_reasoner ${catkin_EXPORTED_TARGETS})
target_link_libraries(visibility_reasoner ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})

## Install executables and/or libraries
install(TARGETS
  pmap_to_octomap visibility_reasoner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)



