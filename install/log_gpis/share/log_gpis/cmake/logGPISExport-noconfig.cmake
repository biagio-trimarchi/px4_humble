#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "log_gpis::logGPIS" for configuration ""
set_property(TARGET log_gpis::logGPIS APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(log_gpis::logGPIS PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblogGPIS.so"
  IMPORTED_SONAME_NOCONFIG "liblogGPIS.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS log_gpis::logGPIS )
list(APPEND _IMPORT_CHECK_FILES_FOR_log_gpis::logGPIS "${_IMPORT_PREFIX}/lib/liblogGPIS.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
