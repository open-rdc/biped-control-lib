# Find OpenRTM-aist
#
# The following additional directories are searched:
# OPENRTM_ROOT (CMake variable)
# OPENRTM_ROOT (Environment variable)
#
# This sets the following variables:
# OPENRTM_FOUND - True if OpenRTM-aist was found.
# OPENRTM_INCLUDE_DIRS - Directories containing the OpenRTM-aist include files.
# OPENRTM_LIBRARIES - Libraries needed to use OpenRTM-aist.
# OPENRTM_CFLAGS - Compiler flags for OpenRTM-aist.
# OPENRTM_VERSION - The version of OpenRTM-aist found.
# OPENRTM_VERSION_MAJOR - The major version of OpenRTM-aist found.
# OPENRTM_VERSION_MINOR - The minor version of OpenRTM-aist found.
# OPENRTM_VERSION_REVISION - The revision version of OpenRTM-aist found.
# OPENRTM_VERSION_CANDIDATE - The candidate version of OpenRTM-aist found.
# OPENRTM_IDL_COMPILER - The IDL compiler used by OpenRTM-aist.
# OPENRTM_IDL_FLAGS - The flags used to compile OpenRTM-aist IDL files.
# OPENRTM_IDL_DIR - The directory containing the OpenRTM-aist IDL files.
#
# This module also defines one macro usable in your CMakeLists.txt files:
# OPENRTM_COMPILE_IDL_FILES(file1 file2 ...)
#   Compiles the specified IDL files, placing the generated C++ source files in
#   ${CMAKE_CURRENT_BINARY_DIR}. The source files can be found in file1_SRCS,
#   file2_SRCS, etc., and all source files for all IDL files are available in
#   ALL_IDL_SRCS. To depend on the generated files, depend on the targets
#   file1_TGT, file2_TGT, etc. The target ALL_IDL_TGT is available to depend on
#   all source files generated from IDL files.

find_package(PkgConfig)
pkg_check_modules(PC_OPENRTM openrtm-aist)
pkg_check_modules(PC_COIL libcoil)
pkg_check_modules(PC_OMNIORB4 omniORB4)
pkg_check_modules(PC_OMNITHREAD3 omnithread3)
pkg_check_modules(PC_OMNIDYNAMIC4 omniDynamic4)
if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
  pkg_check_modules(PC_UUID uuid)
endif ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")

find_path(OPENRTM_INCLUDE_DIR rtm/RTC.h
    HINTS ${OPENRTM_ROOT}/include ${OPENRTM_ROOT} $ENV{OPENRTM_ROOT}/include ${PC_OPENRTM_INCLUDE_DIRS})

find_path(COIL_INCLUDE_DIR coil/config_coil.h
    HINTS ${OPENRTM_ROOT}/include $ENV{OPENRTM_ROOT}/include ${PC_COIL_INCLUDE_DIRS})

find_path(OMNIORB4_INCLUDE_DIR omniORB4/omniORB.h
    HITS ${OMNI_ROOT}/include ${PC_OMNIORB4_INCLUDE_DIRS})

find_library(OPENRTM_LIBRARY RTC
    HINTS ${OPENRTM_ROOT}/lib $ENV{OPENRTM_ROOT}/lib
    ${PC_OPENRTM_LIBRARY_DIRS})
find_library(COIL_LIBRARY coil
    HINTS ${OPENRTM_ROOT}/lib $ENV{OPENRTM_ROOT}/lib
    ${PC_COIL_LIBRARY_DIRS})
find_library(OMNIORB4_LIBRARY omniORB4
    HINTS ${PC_OMNIORB4_LIBRARY_DIRS})
find_library(OMNITHREAD3_LIBRARY omnithread
    HINTS ${PC_OMNITHREAD3_LIBRARY_DIRS})
find_library(OMNIDYNAMIC4_LIBRARY omniDynamic4
    HINTS ${PC_OMNIDYNAMIC4_LIBRARY_DIRS})
if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
  find_library(UUID_LIBRARY uuid
    HINTS ${PC_UUID_LIBRARY_DIRS})
else ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
  set(UUID_LIBRARY "")
endif ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")

set(OPENRTM_CFLAGS ${PC_OPENRTM_CFLAGS_OTHER} ${PC_COIL_CFLAGS_OTHER} ${PC_OMNIORB4_CFLAGS_OTHER})
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIR} ${OPENRTM_INCLUDE_DIR}/rtm/idl
    ${COIL_INCLUDE_DIR} ${OMNIORB4_INCLUDE_DIR})
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARY} ${COIL_LIBRARY} ${OMNIORB4_LIBRARY} ${OMNITHREAD3_LIBRARY} ${OMNIDYNAMIC4_LIBRARY} ${UUID_LIBRARY} dl pthread)

file(STRINGS ${OPENRTM_INCLUDE_DIR}/rtm/version.h OPENRTM_VERSION
    NEWLINE_CONSUME)
#set(OPENRTM_VERSION "1.1.0")
string(REGEX MATCH "version = \"([0-9]+)\\.([0-9]+)\\.([0-9]+)-?([a-zA-Z0-9]*)\""
    OPENRTM_VERSION "${OPENRTM_VERSION}")
set(OPENRTM_VERSION_MAJOR ${CMAKE_MATCH_1})
set(OPENRTM_VERSION_MINOR ${CMAKE_MATCH_2})
set(OPENRTM_VERSION_REVISION ${CMAKE_MATCH_3})
set(OPENRTM_VERSION_CANDIDATE ${CMAKE_MATCH_4})

execute_process(COMMAND rtm-config --libs-only-l OUTPUT_VARIABLE
  OPENRTM_LIBRARIES OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rtm-config --libs-only-L OUTPUT_VARIABLE
  OPENRTM_LIBRARY_DIRS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rtm-config --idlc OUTPUT_VARIABLE OPENRTM_IDL_COMPILER
    OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rtm-config --idlflags OUTPUT_VARIABLE OPENRTM_IDL_FLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
separate_arguments(OPENRTM_IDL_FLAGS)
execute_process(COMMAND rtm-config --prefix OUTPUT_VARIABLE _rtm_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(OPENRTM_IDL_DIR
    ${_rtm_prefix}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    CACHE STRING "Directory containing the OpenRTM-aist IDL files.")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenRTM
    REQUIRED_VARS OPENRTM_INCLUDE_DIR COIL_INCLUDE_DIR OPENRTM_LIBRARY
    COIL_LIBRARY OPENRTM_IDL_COMPILER)

macro(_IDL_OUTPUTS _idl _dir _result)
    set(${_result} ${_dir}/${_idl}SK.cc ${_dir}/${_idl}.hh
        ${_dir}/${_idl}DynSK.cc)
endmacro(_IDL_OUTPUTS)


macro(_COMPILE_IDL _idl_file)
    get_filename_component(_idl ${_idl_file} NAME_WE)
    set(_idl_srcs_var ${_idl}_SRCS)
    _IDL_OUTPUTS(${_idl} ${CMAKE_CURRENT_BINARY_DIR} ${_idl_srcs_var})

    add_custom_command(OUTPUT ${${_idl_srcs_var}}
        COMMAND ${OPENRTM_IDL_COMPILER} ${OPENRTM_IDL_FLAGS}
        -I${OPENRTM_IDL_DIR} ${_idl_file}
        WORKING_DIRECTORY ${CURRENT_BINARY_DIR}
        DEPENDS ${_idl_file}
        COMMENT "Compiling ${_idl_file}" VERBATIM)
    add_custom_target(${_idl}_TGT DEPENDS ${${_idl_srcs_var}})
    set(ALL_IDL_SRCS ${ALL_IDL_SRCS} ${${_idl_srcs_var}})
    if(NOT TARGET ALL_IDL_TGT)
        add_custom_target(ALL_IDL_TGT)
    endif(NOT TARGET ALL_IDL_TGT)
    add_dependencies(ALL_IDL_TGT ${_idl}_TGT)
endmacro(_COMPILE_IDL)

# Module exposed to the user
macro(OPENRTM_COMPILE_IDL_FILES)
    foreach(idl ${ARGN})
        _COMPILE_IDL(${idl})
    endforeach(idl)
endmacro(OPENRTM_COMPILE_IDL_FILES)

