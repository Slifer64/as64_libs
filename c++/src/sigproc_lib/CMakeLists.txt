cmake_minimum_required(VERSION 2.8.3)

project(sigproc_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#########################
######     SPL   ########
#########################

add_library(sigproc_lib
  src/movingAverageFilter.cpp
  src/sigproc.cpp
)

add_dependencies(sigproc_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(sigproc_lib
  ${catkin_LIBRARIES}
  armadillo
)
