cmake_minimum_required(VERSION 2.8.12)
project(nestedidl)

add_definitions(-std=c++11)


message(STATUS "Configuring...")
file(GLOB SOURCES_CXX "*.cxx")
file(GLOB SOURCES_CPP "*.cpp")
file(GLOB SOURCES_CC "*.cc")

SET(BIN_DESTINATION ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DESTINATION})
SET(cmdrDDS_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../../)

include_directories(
${cmdrDDS_INCLUDE_PATH}
)

add_executable(nestedidltest ${SOURCES_CXX} ${SOURCES_CPP} ${SOURCES_CC})
target_link_libraries(nestedidltest fastrtps fastcdr -lpthread)

