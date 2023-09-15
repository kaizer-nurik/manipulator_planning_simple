cmake_minimum_required(VERSION 3.10)
project(Manipulator C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing() #to  discover tests in test explorer 

add_subdirectory(googletest) # add googletest subdirectory

include_directories(googletest/include) # this is so we can #include <gtest/gtest.h>


add_subdirectory(mpnn) # add motion planning nearest neigbourhood directory
include_directories(mpnn/include) # this is so we can #include <gtest/gtest.h>


add_executable(test_IK "./test/test_IK.cpp") # add this executable

include(GoogleTest)
gtest_discover_tests(test_IK) # discovers tests by asking the compiled test executable to enumerate its tests

set(LIB_SOURCE_FILES "./src/obstacles.cpp")
set(LIB_HEADERS_FILES "./include/obstacles.h")
add_library(obstacles STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(obstacles PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/planner.cpp")
set(LIB_HEADERS_FILES "./include/planner.h")
add_library(planner STATIC   ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(planner PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_HEADERS_FILES "./include/robot.h")
set(LIB_SOURCE_FILES "./src/robot.cpp")
add_library(robot STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES})
set_target_properties(robot PROPERTIES LINKER_LANGUAGE CXX) 

set(LIB_HEADERS_FILES "./include/parsing.h")
set(LIB_HEADERS_FILES "./include/parsing.h")
add_library(parsing STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(parsing PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB sources "./rapidxml/*.hpp")
set(LIB_SOURCE_FILES "./src/rapidxml.cpp")
add_library(rapidxml STATIC ${sources} ${LIB_SOURCE_FILES}) 
set_target_properties(rapidxml PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/file_saver.cpp")
set(LIB_HEADERS_FILES "./include/file_saver.h")
add_library(file_saver STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(file_saver PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/inverse_kinematics.cpp")
set(LIB_HEADERS_FILES "./include/inverse_kinematics.h")
add_library(inverse_kinematics STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(inverse_kinematics PROPERTIES LINKER_LANGUAGE CXX)


find_package (Threads)

include_directories(./rapidxml ./include ./googletest)

add_executable(main "./src/main.cpp")
target_link_libraries(main Threads::Threads inverse_kinematics file_saver robot parsing planner rapidxml obstacles DNN )
target_link_libraries(test_IK Threads::Threads inverse_kinematics  gtest robot parsing planner rapidxml obstacles)   
set_target_properties(main PROPERTIES LINKER_LANGUAGE CXX) 



if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()
