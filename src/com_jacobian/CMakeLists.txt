cmake_minimum_required(VERSION 2.6)

project(comjacobian)

set(CMAKE_BUILD_TYPE Debug)

include(FindPkgConfig)
include(CheckIncludeFiles)

set(CMAKE_CXX_FLAGS "-std=c++11")

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_executable(comjacobian com_jacobian_cnoid.cpp)

target_link_libraries(comjacobian ${CHOREONOID_LIBRARIES})
