cmake_minimum_required(VERSION 2.8.3)

project(gmp_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}

  src/GatingFunction/GatingFunction.cpp
  src/GatingFunction/LinGatingFunction.cpp
  src/GatingFunction/ExpGatingFunction.cpp
  src/GatingFunction/SigmoidGatingFunction.cpp

  src/trainMethods/LeastSquares.cpp
  src/trainMethods/LWR.cpp

  src/math/quaternions.cpp
  src/io/io.cpp

  src/WSoG/WSoG.cpp
  src/GMP/GMP.cpp
  src/GMP/GMP_nDoF.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARY}
)
