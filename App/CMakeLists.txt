cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-local-typedefs -Wno-unknown-pragmas -Wno-deprecated")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")
# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

find_package(LIBELAS REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${LIBELAS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
message("libelas dir:" ${LIBELAS_INCLUDE_DIRS})
add_executable( LIBELASApp main.cpp Timer.cpp Timer.h)
target_link_libraries(LIBELASApp ${LIBELAS_LIBRARIES} ${OpenCV_LIBRARIES} )

 
