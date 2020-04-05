cmake_minimum_required(VERSION 3.0.0)

project(lwr4p_test)

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

add_executable(lwr4p_robot_test
  src/lwr4p_robot_test.cpp
  src/utils.cpp
)

add_dependencies(lwr4p_robot_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lwr4p_robot_test
  ${ARMADILLO_LIBRARY}
  lwr4p
  misc_lib
  ${catkin_LIBRARIES}
)

# ===================================

add_executable(multi_lwr4p_robots_test
  src/multi_lwr4p_robots_test.cpp
  src/utils.cpp
)

add_dependencies(multi_lwr4p_robots_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(multi_lwr4p_robots_test
  ${ARMADILLO_LIBRARY}
  lwr4p
  misc_lib
  ${catkin_LIBRARIES}
)

# ===================================
# ===================================
