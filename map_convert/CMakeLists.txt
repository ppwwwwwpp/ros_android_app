cmake_minimum_required(VERSION 2.8.3)
project(map_convert)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dashan_msg
)



catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(base64map src/base64map.cpp)
target_link_libraries(base64map ${catkin_LIBRARIES})
add_dependencies(base64map visual_slam_generate_messages_cpp)
