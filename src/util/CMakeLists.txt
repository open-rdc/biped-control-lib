cmake_minimum_required(VERSION 2.8)

project(testRiccatiEquation)

set(DCMAKE_BUILD_TYPE Debug)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
add_definitions(-lstdc++ -std=c++11)

include_directories(
	${EIGEN3_INCLUDE_DIR}
)

add_executable(testRiccatiEquation testRiccatiEquation.cpp Riccati.h)
