cmake_minimum_required(VERSION 3.24)
project(Mapping_Multi_thread)

set(CMAKE_CXX_STANDARD 14 -O2)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
include_directories( include)
find_package(PCL REQUIRED)

ADD_LIBRARY(sophus SHARED include/sophus/se3.cpp
        include/sophus/so3.cpp
        include/sophus/so3.h
        include/sophus/se3.h)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

ADD_LIBRARY(poseIO SHARED include/g2oIO/PoseGraphIO.h include/g2oIO/PoseGraphIO.cpp)
target_link_libraries(poseIO g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_library(registration
        include/registration/registration.cpp
        include/registration/registration.h
        )
target_link_libraries(registration
        ${PCL_LIBRARIES})

add_executable(Mapping_Multi_thread main.cpp include/mapping.cpp )
target_link_libraries(Mapping_Multi_thread
        poseIO
        sophus
        ${roscpp_LIBRARIES}
        ${PCL_LIBRARIES}
        registration )
