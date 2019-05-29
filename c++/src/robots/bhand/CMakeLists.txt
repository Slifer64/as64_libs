cmake_minimum_required(VERSION 2.8.3)

project(bhand)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing -Wno-write-strings")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

add_definitions(-DLINUX)

find_package(orocos_kdl)
find_package(catkin REQUIRED
  orocos_kdl
  kdl_parser
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

file(GLOB BHAND_API_SRC "src/bhand_API/*.cpp" "include/bhand/bhand_API/src/*.cpp" "include/bhand/bhand_API/src/linux/*.cpp")

SET(SRC_LIST
  src/utils.cpp
  src/kinematic_chain.cpp
  src/robot_hand.cpp
  src/bh282_robot.cpp
  src/bh282_sim_robot.cpp
)

include_directories(
  include
  usr/include/eigen3
  include/bhand/bhand_API
  include/bhand/bhand_API/ctb-0.14
  include/bhand/bhand_API/ctb-0.14/linux
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
  ${BHAND_API_SRC}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pcan
  PocoFoundation
  pthread
  orocos-kdl
  armadillo
  ${catkin_LIBRARIES}
)
