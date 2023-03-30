cmake_minimum_required(VERSION 3.14)
project(erasor_noros
	VERSION 1.0.0
	DESCRIPTION "ERASOR but no ros needed"
	LANGUAGES CXX
)

set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)
find_package(PCL REQUIRED)
add_executable(downsample src/downsample.cpp)
target_link_libraries(downsample ${PCL_LIBRARIES} ${LOG_LIB}) # yaml-cpp  # dufo_lib
