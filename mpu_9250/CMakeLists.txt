cmake_minimum_required(VERSION 3.0)
project(mpu_9250_driver)

add_compile_options(-std=c++11 -frtti)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
	mpu_9250_driver
		mpu_9250_node.cpp
		smbus.c
)

target_link_libraries(
	mpu_9250_driver
		${catkin_LIBRARIES}
)