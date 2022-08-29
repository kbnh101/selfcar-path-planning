cmake_minimum_required(VERSION 3.0.2)
project(parking_path)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES parking_path
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
include/parking_path.h

src/parking_path.cpp
src/parking_path_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
