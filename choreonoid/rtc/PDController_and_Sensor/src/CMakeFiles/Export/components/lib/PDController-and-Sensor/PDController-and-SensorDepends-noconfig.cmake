#----------------------------------------------------------------
# Generated CMake target import file for configuration "".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PDController-and-Sensor" for configuration ""
set_property(TARGET PDController-and-Sensor APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(PDController-and-Sensor PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "pthread;omniORB4;omnithread;omniDynamic4;RTC;coil"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/components/lib/PDController-and-Sensor.so"
  IMPORTED_SONAME_NOCONFIG "PDController-and-Sensor.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS PDController-and-Sensor )
list(APPEND _IMPORT_CHECK_FILES_FOR_PDController-and-Sensor "${_IMPORT_PREFIX}/components/lib/PDController-and-Sensor.so" )

# Import target "PDController-and-SensorComp" for configuration ""
set_property(TARGET PDController-and-SensorComp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(PDController-and-SensorComp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/components/bin/PDController-and-SensorComp"
  )

list(APPEND _IMPORT_CHECK_TARGETS PDController-and-SensorComp )
list(APPEND _IMPORT_CHECK_FILES_FOR_PDController-and-SensorComp "${_IMPORT_PREFIX}/components/bin/PDController-and-SensorComp" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
