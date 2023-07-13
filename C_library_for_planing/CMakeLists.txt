cmake_minimum_required(VERSION 3.10)
project(Manipulator C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")


enable_testing() #to  discover tests in test explorer 

add_subdirectory(googletest) # add googletest subdirectory

include_directories(googletest/include) # this is so we can #include <gtest/gtest.h>

add_executable(mytests "./test/mytests.cpp") # add this executable

include(GoogleTest)
gtest_discover_tests(mytests) # discovers tests by asking the compiled test executable to enumerate its tests

set(LIB_HEADERS_FILES "./include/obstacles.h")
add_library(obstacles STATIC ${LIB_HEADERS_FILES}) 
set_target_properties(obstacles PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_HEADERS_FILES "./include/planner.h")
add_library(planner STATIC ${LIB_HEADERS_FILES}) 
set_target_properties(planner PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_HEADERS_FILES "./include/benchmark.h")
add_library(benchmark STATIC ${LIB_HEADERS_FILES}) 
set_target_properties(benchmark PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_HEADERS_FILES "./include/robot.h")
add_library(robot STATIC ${LIB_HEADERS_FILES})
set_target_properties(robot PROPERTIES LINKER_LANGUAGE CXX) 

set(LIB_HEADERS_FILES "./include/parsing.h")
add_library(parsing STATIC ${LIB_HEADERS_FILES}) 
set_target_properties(parsing PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB sources "./rapidxml/*.hpp")
add_library(rapidxml STATIC ${sources}) 
set_target_properties(rapidxml PROPERTIES LINKER_LANGUAGE CXX)

include_directories(./rapidxml ./include)

add_executable(main "./src/main.cpp")
target_link_libraries(main obstacles robot parsing rapidxml)
target_link_libraries(mytests gtest robot parsing rapidxml)   
set_target_properties(main PROPERTIES LINKER_LANGUAGE CXX) 


if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()

