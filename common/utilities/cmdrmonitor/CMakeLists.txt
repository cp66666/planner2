cmake_minimum_required(VERSION 2.8.3)
project(cmdrmonitor)
add_compile_options(-std=c++11)

SET(cmdrDDS_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../../)
include_directories(
${cmdrDDS_INCLUDE_PATH}
)
add_executable(
cmdrmonitor 
dds_msg_monitor.cpp)

target_link_libraries(cmdrmonitor fastrtps fastcdr -lpthread)