cmake_minimum_required(VERSION 2.8.3)
project(strategy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg 
 	image_transport 
	message_generation
	roscpp
	roslib
	rospy
	sensor_msgs
	std_msgs
	tku_msgs
	tku_libs
)

find_package(OpenCV)
set(TKU_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libtku_libs.so)
set(STRATEGYBASE_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libroscommunication.so)
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
	FILES
	changedisks.msg
)
## Generate services in the 'srv' folder
#add_service_files(DIRECTORY srv 
#	FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES tku_libs roscommunication ros2multicommunication
 	#std_msgs
 	#roscpp
)


