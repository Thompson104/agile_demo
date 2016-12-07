cmake_minimum_required(VERSION 2.8.3)
project(agile_demo_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	sensor_msgs
)

#add_message_files(FILES
#)

add_service_files(FILES
	PublishPointCloud.srv
)

generate_messages(
	DEPENDENCIES geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime geometry_msgs
	LIBRARIES ${PROJECT_NAME}
)

# Make a dummy library to force dependent targets to have dependencies on message generation.
configure_file("/dev/null" "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_library(${PROJECT_NAME} "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
