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

add_executable(motion_planner src/motion_planner.cpp)
add_executable(agile_ik_tests src/ik_tests.cpp)
target_link_libraries(agile_ik_tests
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${orocos_kdl_LIBRARIES}
)

target_link_libraries(motion_planner
	${catkin_LIBRARIES}
)

install(TARGETS agile_ik_tests
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
