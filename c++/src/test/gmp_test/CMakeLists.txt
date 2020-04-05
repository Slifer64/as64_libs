cmake_minimum_required(VERSION 2.8.3)
project(gmp_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
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
    src/utils.cpp
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARY}
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
  ${ARMADILLO_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# ***************************************************
# ************   Test WSoG update  ******************
# ***************************************************

add_executable(test_WSoG_update
  src/test_WSoG_update.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(test_WSoG_update
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_WSoG_update ${LINK_LIBS} )

# **************************************************
# ************   Test GMP update  ******************
# **************************************************

add_executable(test_GMP_update
  src/test_GMP_update.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(test_GMP_update
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})

target_link_libraries(test_GMP_update ${LINK_LIBS} )

# **************************************************
# ************   Test GMP as DMP  ******************
# **************************************************

add_executable(test_gmp_as_dmp
  src/test_gmp_as_dmp.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(test_gmp_as_dmp
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})

target_link_libraries(test_gmp_as_dmp ${LINK_LIBS} )

# **************************************************
# ************   Test n-DoF GMP   ******************
# **************************************************

add_executable(test_nDoF_gmp
        src/test_nDoF_gmp.cpp ${INCLUDE_LIST} ${SRC_LIST}
        )

add_dependencies(test_nDoF_gmp
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_nDoF_gmp ${LINK_LIBS} )
