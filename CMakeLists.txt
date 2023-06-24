cmake_minimum_required(VERSION 3.14)
project(erasor_noros
	VERSION 1.0.0
	DESCRIPTION "ERASOR but no ros needed"
	LANGUAGES CXX
)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

#############
# PACKAGES  #
#############

find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
set(FUN_LIB Eigen3::Eigen)

#############
# LIBRARIES #
#############
include_directories(include)
add_library(dufo_lib SHARED src/MapUpdater.cpp src/ERASOR.cpp)
target_link_libraries(dufo_lib ${LOG_LIB} yaml-cpp ${FUN_LIB} ${PCL_LIBRARIES})

############
# BINARIES #
############
add_executable(erasor_run ERASOR_pcd.cpp)
target_link_libraries(erasor_run dufo_lib)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
	# print message to show the version of OpenMP
	message(STATUS "OpenMP found, version: ${OpenMP_CXX_VERSION_STRING}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    target_link_libraries(erasor_run OpenMP::OpenMP_CXX)
endif()