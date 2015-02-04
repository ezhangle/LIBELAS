cmake_minimum_required( VERSION 3.0 )
project( LIBELAS )

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.)

include(SetPlatformVars)
add_subdirectory(LIBELAS)
add_subdirectory(App)




