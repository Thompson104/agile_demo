cmake_minimum_required(VERSION 2.8.3)
project(agile_demo)

find_package(catkin REQUIRED xacro)

catkin_package()

xacro_add_xacro_file("${CMAKE_CURRENT_SOURCE_DIR}/urdf/demo.urdf.xacro" "${CMAKE_CURRENT_SOURCE_DIR}/urdf/demo.urdf" INORDER)
xacro_add_xacro_file("${CMAKE_CURRENT_SOURCE_DIR}/srdf/demo.srdf.xacro" "${CMAKE_CURRENT_SOURCE_DIR}/srdf/demo.srdf" INORDER)

add_custom_target(${PROJECT_NAME}_urdf ALL DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/urdf/demo.urdf")
add_custom_target(${PROJECT_NAME}_srdf ALL DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/srdf/demo.srdf")

install(
	DIRECTORY urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(
	DIRECTORY srdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
)
