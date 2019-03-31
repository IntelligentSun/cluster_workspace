cmake_minimum_required(VERSION 2.8.3)
project(yikun_cluster_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  roscpp
  rospy
  std_msgs
  nav_msgs
  geographic_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DockPose.msg
  UWB.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetPose.srv
  SetState.srv
  GetState.srv
  SetDatum.srv
  SetPosition.srv
  ToggleFilterProcessing.srv
  SetPath.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  FollowPath.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  actionlib_msgs
  geographic_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yikun_msgs
#  CATKIN_DEPENDS roscpp rospy geographic_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
