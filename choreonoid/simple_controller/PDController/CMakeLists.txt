cmake_minimum_required(VERSION 2.8)

project(PDController)

find_package(Boost COMPONENTS system thread filesystem)

add_definitions(-std=c++11)

include(FindPkgConfig)
pkg_check_modules(cnoid_plugin REQUIRED choreonoid-body-plugin)

include_directories(
	${cnoid_plugin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${cnoid_plugin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

add_library(PDController SHARED PDController.cpp)
set_target_properties(PDController PROPERTIES PREFIX "")

set(target PDController)
target_link_libraries(PDController
	${cnoid_plugin_LIBRARIES}
	${Boost_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

install(TARGETS PDController LIBRARY DESTINATION /usr/lib/choreonoid-1.6/simplecontroller)
install(FILES pdgain.txt DESTINATION /usr/lib/choreonoid-1.6/simplecontroller)
