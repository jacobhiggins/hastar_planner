cmake_minimum_required(VERSION 3.0.2)
project(hastar_planner)

# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include/hastar_planner
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/libs)
add_subdirectory(src/nodes)