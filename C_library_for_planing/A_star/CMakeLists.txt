cmake_minimum_required(VERSION 3.10)
project(A_star C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
include_directories(../rapidxml ../common/include ../common  ../json-develop/include ./include)


set(LIB_SOURCE_FILES "./src/planner.cpp")
set(LIB_HEADERS_FILES "./include/planner.h")
add_library(a_star_planner STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(a_star_planner PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/benchmark.cpp")
set(LIB_HEADERS_FILES "./include/benchmark.h")
add_library(benchmark_a_star STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(benchmark_a_star PROPERTIES LINKER_LANGUAGE CXX)


if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()

