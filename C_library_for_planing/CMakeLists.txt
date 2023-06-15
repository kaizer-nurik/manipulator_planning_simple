cmake_minimum_required(VERSION 3.10)
project(Manipulator C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

set(LIB_HEADERS_FILES "./include/robot.h")
add_library(robot STATIC ${LIB_HEADERS_FILES})
set_target_properties(robot PROPERTIES LINKER_LANGUAGE CXX) 

set(LIB_HEADERS_FILES "./include/parsing.h")
add_library(parsing STATIC ${LIB_HEADERS_FILES}) 
set_target_properties(parsing PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB sources "../rapidxml/*.hpp")
add_library(rapidxml STATIC ${sources}) 
set_target_properties(rapidxml PROPERTIES LINKER_LANGUAGE CXX)

get_filename_component(PARENT_DIR ../ ABSOLUTE)
include_directories(${PARENT_DIR}/rapidxml ./include)

add_executable(main "./src/main.cpp")
target_link_libraries(main obstacles robot parsing rapidxml)
target_link_libraries(mytests gtest robot parsing rapidxml)   
set_target_properties(main PROPERTIES LINKER_LANGUAGE CXX) 

