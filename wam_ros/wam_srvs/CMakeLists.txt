cmake_minimum_required(VERSION 2.8.3)
project(wam_srvs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation genmsg)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  CartPosMove.srv
  GravityComp.srv
  Hold.srv
  JointMove.srv
  OrtnMove.srv
  OrtnSplitMove.srv
  PoseMove.srv
  CartVel.srv
  BHandFingerPos.srv
  BHandFingerVel.srv
  BHandGraspPos.srv
  BHandGraspVel.srv
  BHandSpreadPos.srv
  BHandSpreadVel.srv
  Teach.srv
  Play.srv
  BHandPinchPos.srv
  Link.srv
  ForceTorqueTool.srv
  FollowPath.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)
