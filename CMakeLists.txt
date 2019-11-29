cmake_minimum_required(VERSION 2.8.3)
project(pi_drone_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

#catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    sendRC.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  #   mavros_msgs#   sensor_msgs#   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pi_drone
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


