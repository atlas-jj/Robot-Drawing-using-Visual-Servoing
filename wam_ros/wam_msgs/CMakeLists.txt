cmake_minimum_required(VERSION 2.8.3)
project(wam_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation genmsg)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RTCartPos.msg
  RTCartVel.msg
  RTJointPos.msg
  RTJointVel.msg
  RTOrtnPos.msg
  RTOrtnVel.msg
  RTPose.msg
  MatrixMN.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
)
