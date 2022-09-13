cmake_minimum_required(VERSION 3.0.2)
project(global_path_selfcar_tryout)

find_package(OpenCV REQUIRED)

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
#  LIBRARIES dijkstra
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} 
src/dijkstra.cpp

include/csv.h
include/dijkstra.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
