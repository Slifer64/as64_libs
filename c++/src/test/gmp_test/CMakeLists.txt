cmake_minimum_required(VERSION 2.8.3)
project(gmp_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  gmp_lib
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

)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  gmp_lib
  io_lib
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# *******************************************
# ************   Test gmp  ******************
# *******************************************

add_executable(test_WSoG_cond
  src/test_WSoG_update.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(test_WSoG_cond
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_WSoG_cond ${LINK_LIBS} )
