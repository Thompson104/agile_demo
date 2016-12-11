cmake_minimum_required(VERSION 2.8.3)
project(agile_demo_core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	agile_demo_msgs
	agile_demo_motion
	dr_eigen
	dr_param
	roscpp
	roslib
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	agile_demo_msgs
	agile_demo_motion
	dr_eigen
	dr_param
	roscpp
	roslib
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(coordinator src/coordinator.cpp)

target_link_libraries(coordinator
	${catkin_LIBRARIES}
)
