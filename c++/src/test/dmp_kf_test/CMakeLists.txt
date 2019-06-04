cmake_minimum_required(VERSION 2.8.3)
project(dmp_kf_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dmp_lib
  kf_lib
  io_lib
  math_lib
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
  kf_lib
  io_lib
  math_lib
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

# ********************************************
# ************   Train DMP  ******************
# ********************************************

add_executable(trainDMP
		src/trainDMP.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(trainDMP
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(trainDMP ${LINK_LIBS} )


# ******************************************
# ************   Sim DMP  ******************
# ******************************************

add_executable(simDMP
		src/simDMP.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(simDMP
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(simDMP ${LINK_LIBS} )


# *******************************************************
# ************   sim_DMP_Pos_EKF_disc  ******************
# *******************************************************

add_executable(sim_DMP_Pos_EKF_disc
		src/sim_DMP_Pos_EKF_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(sim_DMP_Pos_EKF_disc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMP_Pos_EKF_disc ${LINK_LIBS} )


# **********************************************************
# ************   sim_DMP_Orient_EKF_disc  ******************
# **********************************************************

add_executable(sim_DMP_Orient_EKF_disc
		src/sim_DMP_Orient_EKF_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(sim_DMP_Orient_EKF_disc
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMP_Orient_EKF_disc ${LINK_LIBS} )