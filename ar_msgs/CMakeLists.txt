cmake_minimum_required(VERSION 2.8.3)
project(ar_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ARMarker.msg
   ARMarkers.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ARDocking.srv
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_msgs
#  CATKIN_DEPENDS message_generation roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

