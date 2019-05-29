cmake_minimum_required(VERSION 2.8.3)

project(misc_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS orocos_kdl
)

SET(SRC_LIST
  src/timer.cpp
  src/joint_state_publisher.cpp
  src/wrench_reader.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pthread
  armadillo
  ${catkin_LIBRARIES}
)
