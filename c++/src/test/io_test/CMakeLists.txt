cmake_minimum_required(VERSION 2.8.3)
project(io_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )

find_package(armadillo REQUIRED)
find_package(eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
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


# *******************************************
# ************   FILE IO   ******************
# *******************************************

add_executable(file_io_test
		src/file_io_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(file_io_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(file_io_test
  ${catkin_LIBRARIES}
  io_lib
)

# ---------------------------------------------

add_executable(file_io_matlab_test
	src/file_io_matlab_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(file_io_matlab_test
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

target_link_libraries(file_io_matlab_test
	${catkin_LIBRARIES}
	io_lib
)



# **********************************************
# ************   General IO   ******************
# **********************************************

add_executable(io_test
        src/io_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(io_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(io_test
  ${catkin_LIBRARIES}
  io_lib
)

# *************************************************
# **********  COLORED TERMINAL OUTPUT  ************
# *************************************************

add_executable(term_color_test
  src/term_color_test.cpp
)

add_dependencies(term_color_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(term_color_test
  ${catkin_LIBRARIES}
  io_lib
)

# *************************************
# **********   XML PARSER  ************
# *************************************

add_executable(xml_parser_test
  src/xml_parser_test.cpp
)

add_dependencies(xml_parser_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(xml_parser_test
  ${catkin_LIBRARIES}
  io_lib
  ${ARMADILLO_LIBRARY}
)
