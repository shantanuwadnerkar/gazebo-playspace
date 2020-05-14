cmake_minimum_required(VERSION 2.8.3)
project(simple_arm)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Check for c++11 / c++0x support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "-std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "-std=c++0x")
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros
	# std_msgs
	# message_generation
	# controller_manager
)

find_package(gazebo REQUIRED COMPONENTS)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
	DEPENDS roscpp gazebo_ros
	# INCLUDE_DIRS include
	# LIBRARIES simple_arm
)

# add_library(hello_world_plugin src/hello_world_plugin.cpp)
# target_link_libraries(hello_world_plugin ${catkin_LIBRARIES} ${gazebo_LIBRARIES})