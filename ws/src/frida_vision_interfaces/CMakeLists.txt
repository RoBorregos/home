cmake_minimum_required(VERSION 3.0.2)
project(frida_vision_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    actionlib
    geometry_msgs
    std_msgs
    std_srvs
    message_generation
)


add_message_files(
  FILES
  Person.msg
  PersonList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  NewHost.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  DetectPointingObject.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frida_vision_interfaces
CATKIN_DEPENDS geometry_msgs message_generation std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
