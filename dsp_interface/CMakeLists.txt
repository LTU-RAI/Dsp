cmake_minimum_required(VERSION 3.5)
project(dsp_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME} "srv/PathCost.srv"
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs message_runtime
#  DEPENDS eigen system_lib
#)

###########
## Build ##
###########

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
#  ${OCTOMAP_INCLUDE_DIR}
#)


## Declare a cpp library
#add_library(dsp
#  src/${PROJECT_NAME}/dsp.cpp
#)

#target_link_libraries(dsp
#  ${DSL_LIBRARY}
#  ${Eigen_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#)

## Declare a cpp executable
#add_executable(dsp_node src/${PROJECT_NAME}/dsp_node.cpp)
#add_executable(path_to_pose src/${PROJECT_NAME}/path_to_pose.py)


## Specify libraries to link a library or executable target against
#target_link_libraries(dsp_node
#  dsp
#  ${catkin_LIBRARIES}
#)
