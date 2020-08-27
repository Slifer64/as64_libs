cmake_minimum_required(VERSION 2.8.3)
project(ur_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  io_lib
  ur_robot
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

SET(BUILD_LIBS
  ${catkin_LIBRARIES}
  armadillo
  io_lib
  ur_robot
)

# *************************************************
# *************************************************

add_executable(test_ur src/test_ur.cpp ${INCLUDE_LIST} ${SRC_LIST} )
add_dependencies(test_ur ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_ur ${BUILD_LIBS} )


# *************************************************
# *************************************************

#add_executable(run_urscript src/run_urscript.cpp ${INCLUDE_LIST} ${SRC_LIST} )
#add_dependencies(run_urscript ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(run_urscript ${BUILD_LIBS} )

# *************************************************
# *************************************************
