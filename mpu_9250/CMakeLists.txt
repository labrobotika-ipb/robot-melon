cmake_minimum_required(VERSION 3.0)
project(mpu_9250)

add_compile_options(-std=c++11 -frtti)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
#	${YAML_CPP_INCLUDE_DIRS}
)

add_executable(
	mpu_9250_driver
		mpu_9250_node.cpp
		mpu_driver.cpp
		smbus.c
)

target_link_libraries(
	mpu_9250_driver
		${catkin_LIBRARIES}
		${YAML_CPP_LIBRARIES}
)