cmake_minimum_required(VERSION 2.8.3)

project(pro)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include) 

add_library(ndt_omp src/pclomp/voxel_grid_covariance_omp.cpp src/pclomp/ndt_omp.cpp src/pclomp/gicp_omp.cpp)

add_executable(pro src/main.cpp)

target_link_libraries(pro ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ndt_omp)
