cmake_minimum_required(VERSION 2.4.6)
project(thermo_blob)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES blob.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(DEPENDS OpenCV)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
##include_directories(/usr/local/include)
##link_directories(${OpenCV_LIB_DIR})

add_executable(arraymsg_pub src/arraymsg_pub.cpp)
target_link_libraries(arraymsg_pub ${catkin_LIBRARIES})
add_dependencies(arraymsg_pub thermo_blob_generate_message_cpp)

add_executable(statemsg_pub src/statemsg_pub.cpp)
target_link_libraries(statemsg_pub ${catkin_LIBRARIES})
add_dependencies(statemsg_pub statemsg_generate_message_cpp)

add_executable(thermo_blob src/thermo_blob.cpp)
target_link_libraries(thermo_blob ${catkin_LIBRARIES})
find_package(OpenCV REQUIRED)
target_link_libraries(thermo_blob ${OpenCV_LIBS})
target_link_libraries(thermo_blob /usr/local/lib/libblob.a)
add_dependencies(thermo_blob thermo_blob_generate_message_cpp)
