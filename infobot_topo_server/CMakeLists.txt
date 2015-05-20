cmake_minimum_required(VERSION 2.8.3)
project(infobot_topo_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  infobot_topo_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES infobot_topo_server
  CATKIN_DEPENDS roscpp infobot_topo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Includes
include_directories(include)
# It is important that SYSTEM is here otherwise, those might not be found in include <>
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS})

# Server node
add_executable(topo_server src/topo_server.cpp)
add_dependencies(topo_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(topo_server yaml-cpp ${catkin_LIBRARIES})

# Saver node
add_executable(topo_saver src/topo_saver.cpp)
add_dependencies(topo_saver ${catkin_EXPORTED_TARGETS})
target_link_libraries(topo_saver yaml-cpp ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS topo_server topo_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

