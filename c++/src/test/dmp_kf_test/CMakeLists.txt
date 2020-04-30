cmake_minimum_required(VERSION 2.8.3)
project(dmp_kf_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dmp_lib
  kf_lib
  io_lib
  math_lib
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
	src/utils.cpp
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARY}
  dmp_lib
  kf_lib
  io_lib
  math_lib
  plot_lib
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

# *****************************************************
# ************   Train DMP position  ******************
# *****************************************************

add_executable(trainDMP_pos
		src/trainDMP_pos.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(trainDMP_pos
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(trainDMP_pos ${LINK_LIBS} )

# ***********************************************
# ************   Train DMP eo  ******************
# ***********************************************

add_executable(trainDMP_eo
		src/trainDMP_eo.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(trainDMP_eo
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(trainDMP_eo ${LINK_LIBS} )


# *********************************************
# ************   Train DMPo  ******************
# *********************************************

add_executable(trainDMP_o
		src/trainDMP_o.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(trainDMP_o
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(trainDMP_o ${LINK_LIBS} )


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

add_executable(sim_DMPpEKFc_disc
		src/sim_DMPpEKFc_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(sim_DMPpEKFc_disc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMPpEKFc_disc ${LINK_LIBS} )


# **********************************************************
# ************   sim_DMPo_EKF_disc  ******************
# **********************************************************

add_executable(sim_DMPoEKFc_disc
		src/sim_DMPoEKFc_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(sim_DMPoEKFc_disc
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMPoEKFc_disc ${LINK_LIBS} )


# ******************************************************
# ************   sim_DMP_eo_EKF_disc  ******************
# ******************************************************

add_executable(sim_DMPeoEKFc_disc
		src/sim_DMPeoEKFc_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(sim_DMPeoEKFc_disc
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMPeoEKFc_disc ${LINK_LIBS} )


# **********************************************************
# ************   sim_DMPoEKFa_disc  ******************
# **********************************************************

add_executable(sim_DMPpEKFa_disc
		src/sim_DMPpEKFa_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
		)

add_dependencies(sim_DMPpEKFa_disc
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMPpEKFa_disc ${LINK_LIBS} )


# **********************************************************
# ************   sim_DMPoEKFa_disc  ******************
# **********************************************************

add_executable(sim_DMPoEKFa_disc
		src/sim_DMPoEKFa_disc.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(sim_DMPoEKFa_disc
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(sim_DMPoEKFa_disc ${LINK_LIBS} )
