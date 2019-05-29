cmake_minimum_required(VERSION 2.8.3)

project(kalmanfilter_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
include
/usr/include/eigen3
${catkin_INCLUDE_DIRS}
)

###################################
######     Kalman Filter   ########
###################################

add_library(${PROJECT_NAME}
  src/KF.cpp
  src/EKF.cpp
  src/UKF.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
)
