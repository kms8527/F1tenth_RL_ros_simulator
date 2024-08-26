cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  geometry_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    gps.msg
    waypoint.msg
    rpy.msg
    obstacle.msg
    waypoint_array.msg
    CarState.msg
    debug.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  global_path.srv
  observation.srv
  reset.srv
  sync_control.srv
  encode_init_state.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
)
