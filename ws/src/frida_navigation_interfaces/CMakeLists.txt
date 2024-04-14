cmake_minimum_required(VERSION 3.0.2)
project(frida_navigation_interfaces)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
)


## Generate actions in the 'action' folder
add_action_files(
  FILES
  navServ.action
)

add_service_files(
  FILES
  ViewAngle.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs move_base_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frida_navigation_interfaces
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation move_base_msgs roscpp rospy std_msgs
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

