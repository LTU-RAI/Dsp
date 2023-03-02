cmake_minimum_required(VERSION 3.0.2)
project(dsp)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  octomap_ros
  octomap_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(DSL REQUIRED)
find_package(octomap REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  pathCost.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs message_runtime
  DEPENDS eigen system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIR}
)


## Declare a cpp library
add_library(dsp
  src/${PROJECT_NAME}/dsp.cpp
)

target_link_libraries(dsp
  ${DSL_LIBRARY}
  ${Eigen_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

## Declare a cpp executable
add_executable(dsp_node src/${PROJECT_NAME}/dsp_node.cpp)
#add_executable(path_to_pose src/${PROJECT_NAME}/path_to_pose.py)


## Specify libraries to link a library or executable target against
target_link_libraries(dsp_node
  dsp
  ${catkin_LIBRARIES}
)
