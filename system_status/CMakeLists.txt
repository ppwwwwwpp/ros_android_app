cmake_minimum_required(VERSION 2.8.3)
project(system_status)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dashan_msg
  bp_std_msgs
)




catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS dashan_msg bp_std_msgs
)

add_executable(system_status src/system_status.cpp)
target_link_libraries(system_status ${catkin_LIBRARIES})
include_directories(/usr/local/include)
link_directories(/usr/lib)

add_dependencies(system_status dashan_msg_gencpp)


