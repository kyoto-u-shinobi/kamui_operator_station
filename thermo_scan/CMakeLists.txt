cmake_minimum_required(VERSION 2.8.3)
project(thermo_scan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
  dynamixel_controllers
#  message_generation
  tf
)

#add_message_files(
#  FILES 
#  thermo_scan.msg
#  Num.msg
#)

#add_service_files(
#   FILES
#   AddTwoInts.srv
#   AddVictim.srv
#   CheckAndGo.srv
# )

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  dynamixel_msgs
#)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thermo_scan src/thermo_scan.cpp)
target_link_libraries(thermo_scan ${catkin_LIBRARIES})
add_dependencies(thermo_scan thermo_scan_generate_message_cpp)

add_executable(thermo_scan2 src/thermo_scan2.cpp)
target_link_libraries(thermo_scan2 ${catkin_LIBRARIES})
add_dependencies(thermo_scan2 thermo_scan_generate_message_cpp)

add_executable(detect_victim_pose_node src/detect_victim_pose.cpp)
target_link_libraries(detect_victim_pose_node ${catkin_LIBRARIES})
add_dependencies(detect_victim_pose_node thermo_scan_generate_message_cpp)
