cmake_minimum_required(VERSION 2.8.3)
project(dmp_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dmp_lib
  io_lib
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

set(SRC_LIST
	src/utils.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ***********************************************
# ************   Test STD DMP  ******************
# ***********************************************

add_executable(dmp_std_test
  src/dmp_std_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_std_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_std_test
  ${catkin_LIBRARIES}
  dmp_lib
  io_lib
)

# *************************************************
# **********  COLORED TERMINAL OUTPUT  ************
# *************************************************


