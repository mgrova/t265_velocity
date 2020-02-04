cmake_minimum_required(VERSION 3.7)
project(t265_velocity)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED roscpp geometry_msgs sensor_msgs nav_msgs)
if(${catkin_FOUND})
	catkin_package(
		CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs
		INCLUDE_DIRS include
	)
endif(${catkin_FOUND})

find_package(Eigen3 REQUIRED)

set(SOURCE_FILES src/main.cpp src/VelocityEstimator.cpp include/t265_velocity/VelocityEstimator.h)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} )
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif(${catkin_FOUND})

if(${Eigen3_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif()