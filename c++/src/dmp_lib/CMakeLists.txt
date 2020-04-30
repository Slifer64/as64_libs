cmake_minimum_required(VERSION 2.8.3)

project(dmp_lib)

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
  src/CanonicalClock/CanonicalClock.cpp

  src/GatingFunction/GatingFunction.cpp
  src/GatingFunction/LinGatingFunction.cpp
  src/GatingFunction/ExpGatingFunction.cpp
  src/GatingFunction/SigmoidGatingFunction.cpp

  src/trainMethods/LeastSquares.cpp
  src/trainMethods/LWR.cpp

  src/math/quaternions.cpp
  src/io/io.cpp

  src/DMP/DMP_.cpp
  src/DMP/DMP.cpp
  src/DMP/DMP_bio.cpp
  src/DMP/DMP_pos.cpp
  src/DMP/DMP_eo.cpp
  src/DMP/DMPo.cpp

  src/KalmanFilter/DMPpEKFa.cpp
  src/KalmanFilter/DMPoEKFa.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARY}
)
