cmake_minimum_required(VERSION 3.0)
project( LIBELAS )

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})
string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

# make release version
set(CMAKE_BUILD_TYPE Release)

message("Build ELAS with OpenMP version")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp -msse3 -std=c++11" )
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

#find_package(LIBELAS REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(HAL REQUIRED)
#find_package(PbMsgs REQUIRED)
#find_package(Calibu REQUIRED)
#find_package(Kangaroo REQUIRED)
#find_package(Eigen3 REQUIRED)

#include_directories(${HAL_INCLUDE_DIRS})
#include_directories(${PbMsgs_INCLUDE_DIRS})
#include_directories(${Calibu_INCLUDE_DIRS})
#include_directories(${Kangaroo_INCLUDE_DIRS})
#include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(${OpenCV_INCLUDE_DIRS})



################################################################################
list(APPEND HDRS
     src/descriptor.h
     src/elas.h
     src/filter.h
     src/matrix.h
     src/triangle.h
)

list(APPEND SRCS
     src/descriptor.cpp
     src/elas.cpp
     src/filter.cpp
     src/matrix.cpp
     src/triangle.cpp
)

#######################################################
#######################################################
#######################################################
## Create configure file for inclusion in library

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

# build demo program
add_library(${LIBRARY_NAME}  ${HDRS} ${SRCS})
target_link_libraries( ${LIBRARY_NAME}
#${OpenCV_LIBRARIES}
#${Calibu_LIBRARIES}
#${HAL_LIBRARIES}
#${PbMsgs_LIBRARIES}
#${Kangaroo_LIBRARIES}
)

#######################################################

# This relative path allows installed files to be relocatable.
set( CMAKECONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}" )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

# Export library for easy inclusion from other cmake projects.
export( TARGETS ${LIBRARY_NAME}
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )

# Version information
configure_file("${PROJECT_NAME}ConfigVersion.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Build tree config
set( EXPORT_LIB_INC_DIR "${LIB_INC_DIR}" )
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY IMMEDIATE )

# Install tree config
set( EXPORT_LIB_INC_DIR "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY )

# Add package to CMake package registery for use from the build tree
export( PACKAGE ${PROJECT_NAME} )

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)
install(FILES ${SRC_H}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)
install(TARGETS ${LIBRARY_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#######################################################
## Install CMake config

INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

install( EXPORT "${PROJECT_NAME}Targets" DESTINATION ${CMAKECONFIG_INSTALL_DIR} )
