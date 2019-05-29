cmake_minimum_required(VERSION 3.0.0)

project(lwr4p_bhand_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

add_definitions(-DLINUX)

find_package(catkin REQUIRED
  roscpp
  roslib
  lwr4p
  bhand
  misc_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# ===============================
# ======   LWR4+ robot   ========
# ===============================

add_executable(lwr4p_bh282_robot_test
  src/lwr4p_bh282_robot_test.cpp
  src/utils.cpp
)

add_dependencies(lwr4p_bh282_robot_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lwr4p_bh282_robot_test
  armadillo
  lwr4p
  bhand
  misc_lib
  ${catkin_LIBRARIES}
)

# ===================================
# ===================================
