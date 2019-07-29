cmake_minimum_required(VERSION 2.8.3)
project(mono_rat_viso)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  image_transport
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mono_rat_viso src/mono_rat_node.cpp)
target_link_libraries(mono_rat_viso
  ${catkin_LIBRARIES}
)
