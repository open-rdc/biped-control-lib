# PDController_and_Sensor CMake config file
#
# This file sets the following variables:
# PDController_and_Sensor_FOUND - Always TRUE.
# PDController_and_Sensor_INCLUDE_DIRS - Directories containing the PDController_and_Sensor include files.
# PDController_and_Sensor_IDL_DIRS - Directories containing the PDController_and_Sensor IDL files.
# PDController_and_Sensor_LIBRARIES - Libraries needed to use PDController_and_Sensor.
# PDController_and_Sensor_DEFINITIONS - Compiler flags for PDController_and_Sensor.
# PDController_and_Sensor_VERSION - The version of PDController_and_Sensor found.
# PDController_and_Sensor_VERSION_MAJOR - The major version of PDController_and_Sensor found.
# PDController_and_Sensor_VERSION_MINOR - The minor version of PDController_and_Sensor found.
# PDController_and_Sensor_VERSION_REVISION - The revision version of PDController_and_Sensor found.
# PDController_and_Sensor_VERSION_CANDIDATE - The candidate version of PDController_and_Sensor found.

message(STATUS "Found PDController_and_Sensor-1.0.0")
set(PDController_and_Sensor_FOUND TRUE)

find_package(<dependency> REQUIRED)

#set(PDController_and_Sensor_INCLUDE_DIRS
#    "/usr/local/include/pdcontroller_and_sensor-1"
#    ${<dependency>_INCLUDE_DIRS}
#    )
#
#set(PDController_and_Sensor_IDL_DIRS
#    "/usr/local/include/pdcontroller_and_sensor-1/idl")
set(PDController_and_Sensor_INCLUDE_DIRS
    "/usr/local/include/"
    ${<dependency>_INCLUDE_DIRS}
    )
set(PDController_and_Sensor_IDL_DIRS
    "/usr/local/include//idl")


if(WIN32)
    set(PDController_and_Sensor_LIBRARIES
        "/usr/local/components/lib/libpdcontroller_and_sensor.a"
        ${<dependency>_LIBRARIES}
        )
else(WIN32)
    set(PDController_and_Sensor_LIBRARIES
        "/usr/local/components/lib/libpdcontroller_and_sensor.so"
        ${<dependency>_LIBRARIES}
        )
endif(WIN32)

set(PDController_and_Sensor_DEFINITIONS ${<dependency>_DEFINITIONS})

set(PDController_and_Sensor_VERSION 1.0.0)
set(PDController_and_Sensor_VERSION_MAJOR 1)
set(PDController_and_Sensor_VERSION_MINOR 0)
set(PDController_and_Sensor_VERSION_REVISION 0)
set(PDController_and_Sensor_VERSION_CANDIDATE )

