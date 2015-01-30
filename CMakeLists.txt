cmake_minimum_required( VERSION 3.0 )
project( LIBELAS )

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.)

include(SetPlatformVars)

# Build Configures
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra  -Wno-unused-function -Wno-unused-parameter -Wno-unknown-pragmas  ")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

add_subdirectory(LIBELAS)
add_subdirectory(App)




