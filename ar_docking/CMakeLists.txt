
cmake_minimum_required(VERSION 2.8)

project(ar_docking)

find_package(catkin REQUIRED
	roscpp
	visualization_msgs
	artoolkit
	geometry_msgs
	tf
	resource_retriever
	image_transport
	cv_bridge
	ar_pose
	camera_info_manager
	npb
	ar_msgs
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(ar_docking
	src/ar_docking.cpp
	src/object.cpp
)
target_link_libraries(ar_docking
	${catkin_LIBRARIES}
)
add_dependencies(ar_docking
	${PROJECT_NAME}_generate_messages_cpp
	${artoolkit_EXPORTED_TARGETS}
)
