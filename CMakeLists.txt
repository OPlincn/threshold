cmake_minimum_required(VERSION 3.0.2)
project(threshold)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Threshold.cfg
  cfg/Contrast.cfg
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_thresholding
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/threshold_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for contrast_node
add_executable(contrast_node src/contrast_node.cpp)
add_dependencies(contrast_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(contrast_node
  ${catkin_LIBRARIES}
)

