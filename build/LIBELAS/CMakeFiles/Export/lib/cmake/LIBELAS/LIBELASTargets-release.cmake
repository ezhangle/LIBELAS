#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libelas" for configuration "Release"
set_property(TARGET libelas APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libelas PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/liblibelas.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS libelas )
list(APPEND _IMPORT_CHECK_FILES_FOR_libelas "/usr/local/lib/liblibelas.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
