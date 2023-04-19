cmake_minimum_required(VERSION 3.24)
project(Mapping_Multi_thread)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

add_executable(Mapping_Multi_thread main.cpp)
target_link_libraries(Mapping_Multi_thread
        ${roscpp_LIBRARIES}
        ${PCL_LIBRARIES})
