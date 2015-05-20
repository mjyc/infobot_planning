cmake_minimum_required(VERSION 2.8.3)
project(infobot_pmap_to_octomap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
  infobot_mapping_msgs
  infobot_octomap_store
  std_msgs
  message_generation
)
find_package(octomap REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_service_files(
  DIRECTORY srv
  FILES
  GetProbabilityOctomapSurface.srv
  GetProbabilityOctomapHeight.srv
)

generate_messages(DEPENDENCIES octomap_msgs infobot_mapping_msgs infobot_octomap_store std_msgs)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS roscpp octomap_msgs infobot_mapping_msgs infobot_octomap_store std_msgs message_runtime
    DEPENDS octomap
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(pmap_to_octomap src/pmap_to_octomap_node.cpp)
add_dependencies(pmap_to_octomap ${catkin_EXPORTED_TARGETS})
add_dependencies(pmap_to_octomap ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pmap_to_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Install executables and/or libraries
install(TARGETS
  pmap_to_octomap
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ## Install project namespaced headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)
