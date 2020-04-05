cmake_minimum_required(VERSION 2.8.3)
project(sigproc_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sigproc_lib
  plot_lib
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
	""
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ARMADILLO_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# *********************************************
# ************   Plot test   ******************
# *********************************************

add_executable(movingAverageFilter_test
  src/movingAverageFilter_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(movingAverageFilter_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(movingAverageFilter_test
  ${catkin_LIBRARIES}
  plot_lib
  sigproc_lib
  ${ARMADILLO_LIBRARY}
)
