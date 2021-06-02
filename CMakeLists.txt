cmake_minimum_required(VERSION 3.0.2)
project(sclerp_motion_planner_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  urdf
  kdl_parser
  trajectory_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
)

link_directories(../../devel/lib)
find_package(kinlib) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sclerp_motion_planner_plugin
  DEPENDS EIGEN3
)

include_directories(
  SYSTEM
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${kinlib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sclerp_interface.cpp
  src/sclerp_planner_manager.cpp
  src/sclerp_planning_context.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  kinlib
)
