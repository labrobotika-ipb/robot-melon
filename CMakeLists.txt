cmake_minimum_required(VERSION 3.0)
project(robot_melon)

add_compile_options(-std=c++11 -frtti)

find_package(Boost REQUIRED COMPONENTS system thread regex)
find_package(Eigen3 REQUIRED)

include_directories(
	include
	${EIGEN3_INCLUDE_DIRS}
)

add_library(
	s6h4 STATIC
		src/arm_commands.cpp
)

add_executable(
	test_arm
		src/test_arm.cpp
)

target_link_libraries(
	test_arm
		s6h4
		Boost::system
		Boost::thread
)