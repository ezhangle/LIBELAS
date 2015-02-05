cmake_minimum_required(VERSION 3.0)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp -msse3 -std=c++11")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

find_package(LIBELAS REQUIRED)
find_package(OpenCV REQUIRED)
find_package(HAL REQUIRED)
find_package(PbMsgs  REQUIRED)
find_package(Calibu REQUIRED)
find_package(Kangaroo REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${HAL_INCLUDE_DIRS})
include_directories(${PbMsgs_INCLUDE_DIRS})
include_directories(${Calibu_INCLUDE_DIRS})
include_directories(${Kangaroo_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories( ${LIBELAS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable( LIBELASApp main.cpp Timer.cpp Timer.h ELASStereo.h ELASStereo.cpp)
target_link_libraries(LIBELASApp
${LIBELAS_LIBRARIES}
${OpenCV_LIBRARIES}
${Calibu_LIBRARIES}
${HAL_LIBRARIES}
${PbMsgs_LIBRARIES}
${Kangaroo_LIBRARIES})

 
