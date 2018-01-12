cmake_minimum_required(VERSION 2.8)

project(TestKinematics)

find_package(Boost COMPONENTS system thread filesystem)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")

include_directories(
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

link_directories(
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
)

add_executable(TestKinematics Kinematics.cpp Jacobian.cpp Link.cpp testKinematics.cpp Kinematics.h Link.h)

target_link_libraries(TestKinematics ${Boost_LIBRARIES})

install(TARGETS TestKinematics RUNTIME DESTINATION bin)
