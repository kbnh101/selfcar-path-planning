cmake_minimum_required(VERSION 3.0.2)
project(school_path)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES school_path
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
include/school_path.h

src/school_path.cpp
src/school_path_node.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
