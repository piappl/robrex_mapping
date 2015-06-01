cmake_minimum_required(VERSION 2.8.3)
project(surfel_mapper_library)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS} -std=c++11)

add_library(surfelmapper STATIC src/surfel_mapper.cpp src/logger.cpp)

target_include_directories(surfelmapper PUBLIC include)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(surfelmapper
   ${PCL_LIBRARIES}
)
