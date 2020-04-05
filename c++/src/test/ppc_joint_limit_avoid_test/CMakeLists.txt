cmake_minimum_required(VERSION 3.0.0)

project(ppc_joint_limit_avoid_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
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

add_executable(ppc_joint_limit_avoid_test
  src/ppc_joint_limit_avoid_test.cpp
)

add_dependencies(ppc_joint_limit_avoid_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ppc_joint_limit_avoid_test
  ${ARMADILLO_LIBRARY}
  lwr4p
  misc_lib
  robo_lib
  ${catkin_LIBRARIES}
)

# ===================================
# ===================================
