cmake_minimum_required(VERSION 2.8)

project(ggp_test)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -Wall -fprofile-arcs -ftest-coverage -O0")

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system filesystem iostreams)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(ggp_test
	main.cpp PreviewControl.cpp FootStepPlanner.cpp GaitPatternGenerator.cpp
) 

target_link_libraries(ggp_test ${Boost_LIBRARIES})

