cmake_minimum_required(VERSION 2.8.7)
project(sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O2 -march=core2")
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(ROOT    "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(msgs)

include_directories(
    /usr/local/include/
    ./msgs
    ${ROOT}/common
    ${ROOT}/common/processheartbeat
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    /usr/local/lib/
    ${ROOT}/common/lib
)

file(GLOB SOURCES
    "*.cpp"
    "*.cxx"
    )

set( PROJECT_LINK_LIBS
    fastcdr
    fastrtps
    ${OpenCV_LIBRARIES}
)
add_executable(planner2 ${SOURCES} $<TARGET_OBJECTS:planner_msg>)
target_link_libraries(planner2 ${PROJECT_LINK_LIBS})
