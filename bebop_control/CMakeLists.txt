cmake_minimum_required(VERSION 2.8.3)
project(bebop_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
## OpenCV package
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
## OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(sumo2bebop1 src/sumo2bebop1.cpp)
target_link_libraries(sumo2bebop1 ${catkin_LIBRARIES})
add_dependencies(sumo2bebop1 sumo2bebop1_generate_messages_cpp)

add_executable(singleDrone src/singleDrone.cpp)
target_link_libraries(singleDrone ${catkin_LIBRARIES})
add_dependencies(singleDrone singleDrone_generate_messages_cpp)

add_executable(Consensustest src/Consensustest.cpp)
target_link_libraries(Consensustest ${catkin_LIBRARIES})
add_dependencies(Consensustest Consensustest_generate_messages_cpp)
target_link_libraries(Consensustest ${OpenCV_LIBRARIES})
