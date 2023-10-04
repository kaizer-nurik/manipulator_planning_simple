cmake_minimum_required(VERSION 3.10)
project(RRT C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RELEASE_FLAGS "-O3")
set(DEBUG_FLAGS "-O3")

set(CMAKE_CXX_FLAGS_RELEASE ${RELEASE_FLAGS})
set(CMAKE_C_FLAGS_RELEASE ${RELEASE_FLAGS})

set(CMAKE_CXX_FLAGS_DEBUG ${DEBUG_FLAGS})
set(CMAKE_C_FLAGS_DEBUG ${DEBUG_FLAGS})


set(LIB_SOURCE_FILES "./src/planner.cpp")
set(LIB_HEADERS_FILES "./include/planner.h")
add_library(planner_RRT STATIC   ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(planner_RRT PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/file_saver.cpp")
set(LIB_HEADERS_FILES "./include/file_saver.h")
add_library(file_saver STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(file_saver PROPERTIES LINKER_LANGUAGE CXX)



find_package (Threads)

include_directories( ./include ../common/include ../mpmm/include)


if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()
