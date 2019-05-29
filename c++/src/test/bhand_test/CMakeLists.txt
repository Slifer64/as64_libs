cmake_minimum_required(VERSION 3.0.0)

project(bhand_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

find_package(catkin REQUIRED
  roscpp
  roslib
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

set(LINK_LIBRARIES
  armadillo
  bhand
  misc_lib
  ${catkin_LIBRARIES}
)

# ===============================
# ======   LWR4+ robot   ========
# ===============================

add_executable(bh282_robot_test
  src/bh282_robot_test.cpp
  src/utils.cpp
)

add_dependencies(bh282_robot_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bh282_robot_test
  ${LINK_LIBRARIES}
)

# ===================================

add_executable(multi_bh282_robots_test
  src/multi_bh282_robots_test.cpp
  src/utils.cpp
)

add_dependencies(multi_bh282_robots_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(multi_bh282_robots_test
  ${LINK_LIBRARIES}
)

# ===================================
# ===================================
