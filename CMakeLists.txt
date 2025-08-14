cmake_minimum_required(VERSION 3.0.2)
project(pose_measurement)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.10 REQUIRED COMPONENTS common io features filters registration segmentation)
find_package(Eigen3 REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs pcl_conversions pcl_ros tf2 tf2_ros tf2_geometry_msgs
  DEPENDS PCL EIGEN3
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Add library
add_library(${PROJECT_NAME}
  src/pose_estimator.cpp
  src/measurement_calculator.cpp
  src/point_cloud_processor.cpp
)

## Link libraries to our library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Add executable
add_executable(pose_measurement_node src/pose_measurement_node.cpp)

## Link executable
target_link_libraries(pose_measurement_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)