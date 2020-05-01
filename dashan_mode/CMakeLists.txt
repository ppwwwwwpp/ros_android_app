cmake_minimum_required(VERSION 2.8.3)
project(dashan_mode)


find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dashan_msg
  costmap_2d
  dynamic_reconfigure
  rospy
  std_msgs
  message_generation
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)



#find_package(yaml-cpp REQUIRED)


find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS YAML_CPP
    CATKIN_DEPENDS message_runtime costmap_2d dynamic_reconfigure roscpp rospy std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS} 
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}  ${YAMLCPP_INCLUDE_DIRS}
  CATKIN_DEPENDS dashan_msg
)


link_directories(${catkin_LIBRARY_DIRS})

add_executable(dashan_mode src/dashan_mode.cpp)
target_link_libraries(dashan_mode yaml-cpp SDL SDL_image ${catkin_LIBRARIES})

add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image ${Boost_LIBRARIES})



#add_dependencies(bp_nomotion_location_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(bp_nomotion_location_node 
#  ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp SDL SDL_image  ${YAMLCPP_LIBRARIES} 
#)




