cmake_minimum_required(VERSION 3.0.2)
project(frida_manipulation_interfaces)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  Gripper.srv
#  EnableOctomap.srv
)

# add_message_files(
#   FILES
#   img.msg
#   img_list.msg
#   move.msg
# )

add_action_files(
  FILES
  manipulationPickAndPlace.action
  MoveJoint.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    roslib
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=shadow")

# install(PROGRAMS
#         scripts/pick_and_place_server.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_executable(test_c scripts/test_c.cpp)
#target_link_libraries(test_c ${catkin_LIBRARIES})
#add_dependencies(test_c ${catkin_EXPORTED_TARGETS})

#add_executable(test_place scripts/test_place.cpp)
#target_link_libraries(test_place ${catkin_LIBRARIES})
#add_dependencies(test_place ${catkin_EXPORTED_TARGETS})
