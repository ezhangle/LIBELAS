# Compute paths
get_filename_component( PROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( LIBELAS_INCLUDE_DIRS "${LIBELAS_CMAKE_DIR}/../../../include;" )

# Library dependencies (contains definitions for IMPORTED targets)
if( NOT TARGET libelas AND NOT LIBELAS_BINARY_DIR )
  include( "${PROJECT_CMAKE_DIR}/LIBELASTargets.cmake" )
endif()

SET( LIBELAS_LIBRARIES "libelas" )
