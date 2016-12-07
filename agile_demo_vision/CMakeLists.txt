cmake_minimum_required(VERSION 2.8.3)
project(agile_demo_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	agile_demo_msgs
	dr_eigen
	dr_param
	pcl_conversions
	pcl_ros
	roscpp
	roslib
	sensor_msgs
	std_srvs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	dr_eigen
	dr_param
	pcl_conversions
	pcl_ros
	roscpp
	roslib
	sensor_msgs
	std_srvs
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)

target_link_libraries(pointcloud_publisher
	${catkin_LIBRARIES}
)

