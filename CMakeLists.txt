cmake_minimum_required(VERSION 2.8)

project(compter_vision_work)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w  -std=c++11 -O0 -g -ggdb")


set(OpenCVFindPath /usr/local/alternative/OpenCV/OpenCV2.0)
find_package(OpenCV REQUIRED
			 PATHS ${OpenCVFindPath})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#include Eigen
include_directories(/usr/local/include/eigen3)


# add_subdirectory(src)
add_subdirectory(sfm sfm)
add_subdirectory(test test)
add_subdirectory(work work)