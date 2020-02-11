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

set(LINK_LIBS
  ${catkin_LIBRARIES}
  dmp_lib
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
# ************   Test DMP  ******************
# *******************************************

add_executable(dmp_test
		src/dmp_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_test ${LINK_LIBS} )


# ***********************************************
# ************   Test DMP_pos  ******************
# ***********************************************

add_executable(dmp_pos_test
	src/dmp_pos_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_pos_test
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_pos_test ${LINK_LIBS} )


# ***********************************************
# ************   Test DMPo  ******************
# ***********************************************

add_executable(dmpo_test
		src/dmpo_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(dmpo_test
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(dmpo_test ${LINK_LIBS} )


# ***********************************************
# ************   Test DMP_eo  ******************
# ***********************************************

add_executable(dmp_eo_test
	src/dmp_eo_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_eo_test
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_eo_test ${LINK_LIBS} )