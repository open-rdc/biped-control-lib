cmake_minimum_required(VERSION 2.8)

project( kinematics_demo)

set(PROJECT_GL demoKinematicsGL)
set(GL_SRC main.cpp)

set(PROJECT_TEST testKinematics)
set(TEST_SRC testKinematics.cpp)

set(DCMAKE_BUILD_TYPE Debug)

find_package(GLUT)
find_package(OpenGL)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system thread filesystem timer)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)
add_definitions(-lglut -lGLU -lGL -lm)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_GL} ${GL_SRC} Link.cpp Kinematics.cpp Jacobian.cpp)
add_executable(${PROJECT_TEST} ${TEST_SRC} Link.cpp Kinematics.cpp Jacobian.cpp)

target_link_libraries(${PROJECT_GL} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
target_link_libraries(${PROJECT_TEST} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})

install(TARGETS ${PROJECT_GL} RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_TEST} RUNTIME DESTINATION bin)
