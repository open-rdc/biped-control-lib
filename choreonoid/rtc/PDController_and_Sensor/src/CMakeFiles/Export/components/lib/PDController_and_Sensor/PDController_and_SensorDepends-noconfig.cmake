#----------------------------------------------------------------
# Generated CMake target import file for configuration "".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PDController_and_Sensor" for configuration ""
set_property(TARGET PDController_and_Sensor APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(PDController_and_Sensor PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "pthread;omniORB4;omnithread;omniDynamic4;RTC;coil;CnoidBody;CnoidBodyPlugin;CnoidUtil;CnoidBase;QtOpenGL;QtGui;QtCore;boost_system;boost_filesystem"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/components/lib/PDController_and_Sensor.so"
  IMPORTED_SONAME_NOCONFIG "PDController_and_Sensor.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS PDController_and_Sensor )
list(APPEND _IMPORT_CHECK_FILES_FOR_PDController_and_Sensor "${_IMPORT_PREFIX}/components/lib/PDController_and_Sensor.so" )

# Import target "PDController_and_SensorComp" for configuration ""
set_property(TARGET PDController_and_SensorComp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(PDController_and_SensorComp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/components/bin/PDController_and_SensorComp"
  )

list(APPEND _IMPORT_CHECK_TARGETS PDController_and_SensorComp )
list(APPEND _IMPORT_CHECK_FILES_FOR_PDController_and_SensorComp "${_IMPORT_PREFIX}/components/bin/PDController_and_SensorComp" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
