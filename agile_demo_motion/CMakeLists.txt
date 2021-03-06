cmake_minimum_required(VERSION 2.8.3)
project(agile_demo_motion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	actionlib
	agile_demo_msgs
	control_msgs
	dr_eigen
	dr_param
	eigen_conversions
	roscpp
	roslib
	std_srvs
	trajectory_msgs
	trac_ik_lib
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	actionlib
	agile_demo_msgs
	control_msgs
	dr_eigen
	dr_param
	eigen_conversions
	roscpp
	roslib
	std_srvs
	trajectory_msgs
	trac_ik_lib
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/motion_planner.cpp
	src/grasp_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)
