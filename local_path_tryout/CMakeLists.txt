cmake_minimum_required(VERSION 3.0.2)
project(local_path_tryout)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  morai_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  morai_msgs
  data_transfer_msg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES loca_path
  CATKIN_DEPENDS ackermann_msgs geometry_msgs morai_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
 include/local_path.h
 include/pure_pursuit.h
 include/pid.h
 include/ros_data.h
 
 src/local_path.cpp
 src/pure_pursuit.cpp
 src/pid.cpp
 src/ros_data.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
