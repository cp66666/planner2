cmake_minimum_required(VERSION 2.8.7)
project(planner_msg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

if(APPLE)
    include_directories(
        /usr/local/include/
    )
endif(APPLE)

file(GLOB SOURCES
    "*.cxx"
    )

add_library(${PROJECT_NAME} OBJECT ${SOURCES})