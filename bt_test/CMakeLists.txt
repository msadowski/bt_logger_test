cmake_minimum_required(VERSION 3.0.2)
project(bt_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bt_test
  CATKIN_DEPENDS behaviortree_cpp_v3 roscpp
#  DEPENDS system_lib
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_executable(bt_test src/bt_test.cpp src/dummy_nodes.cpp)
add_dependencies(bt_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt_test ${catkin_LIBRARIES})
