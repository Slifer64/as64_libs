cmake_minimum_required(VERSION 3.0.0)

project(singular_value_filter_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
add_definitions(-DLINUX)

find_package(catkin REQUIRED
  roscpp
  roslib
  lwr4p
  misc_lib
  robo_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# ===============================
# ======   LWR4+ robot   ========
# ===============================

add_executable(singular_value_filter_test
  src/singular_value_filter_test.cpp
)

add_dependencies(singular_value_filter_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(singular_value_filter_test
  ${ARMADILLO_LIBRARY}
  lwr4p
  misc_lib
  robo_lib
  ${catkin_LIBRARIES}
)

# ===================================
# ===================================
