cmake_minimum_required(VERSION 2.8.3)
project(infobot_topo_store)

find_package(catkin REQUIRED
  roscpp
  infobot_topo_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES infobot_topo_server
  CATKIN_DEPENDS roscpp infobot_topo_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(topo_store src/topo_store.cpp)
add_dependencies(topo_store ${catkin_EXPORTED_TARGETS})
target_link_libraries(topo_store yaml-cpp ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS topo_store
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
