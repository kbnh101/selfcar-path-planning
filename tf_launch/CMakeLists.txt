cmake_minimum_required(VERSION 3.0.2)
project(selfcar_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES selfcar_launch
  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_make
 src/tf.cpp
)

target_link_libraries(tf_make ${catkin_LIBRARIES})


