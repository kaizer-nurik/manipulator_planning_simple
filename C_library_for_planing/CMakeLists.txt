cmake_minimum_required(VERSION 3.10)
project(Manipulator C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(RELEASE_FLAGS "-O3")
set(DEBUG_FLAGS "-O3")

set(CMAKE_CXX_FLAGS_RELEASE ${RELEASE_FLAGS})
set(CMAKE_C_FLAGS_RELEASE ${RELEASE_FLAGS})

set(CMAKE_CXX_FLAGS_DEBUG ${DEBUG_FLAGS})
set(CMAKE_C_FLAGS_DEBUG ${DEBUG_FLAGS})

enable_testing() #to  discover tests in test explorer 
add_subdirectory(googletest) # add googletest subdirectory
include_directories(googletest/include) # this is so we can #include <gtest/gtest.h>
# add_executable(test_IK "./test/test_IK.cpp") # add this executable
# gtest_discover_tests(test_IK) # discovers tests by asking the compiled test executable to enumerate its tests

add_subdirectory(mpnn) # add motion planning nearest neigbourhood directory
include_directories(mpnn/include) # this is so we can #include <ANN.h>

add_subdirectory(json-develop) 
include_directories(json-develop/single_include/nlohmann)


add_subdirectory(common) 
include_directories(common/include)

add_subdirectory(RRT) 
include_directories(RRT/include)

add_subdirectory(A_star) 
include_directories(A_star/include)

file(GLOB sources "./rapidxml/*.hpp")
add_library(rapidxml STATIC ${sources}) 
set_target_properties(rapidxml PROPERTIES LINKER_LANGUAGE CXX)

find_package(Threads)

include_directories(./rapidxml ./googletest ./include ../common/include ../mpmm/include)

add_executable(main_a_star "./main_a_star.cpp")
target_link_libraries(main_a_star Threads::Threads inverse_kinematics file_saver  parsing robot a_star_planner rapidxml obstacles DNN )
set_target_properties(main_a_star PROPERTIES LINKER_LANGUAGE CXX) 

add_executable(main_a_star_single "./main_a_star_single.cpp")
target_link_libraries(main_a_star_single Threads::Threads inverse_kinematics file_saver  parsing robot a_star_planner rapidxml obstacles DNN )
set_target_properties(main_a_star_single PROPERTIES LINKER_LANGUAGE CXX) 

add_executable(main_rrt "./main_rrt.cpp")
target_link_libraries(main_rrt Threads::Threads inverse_kinematics file_saver  parsing robot planner_RRT rapidxml obstacles DNN )
set_target_properties(main_rrt PROPERTIES LINKER_LANGUAGE CXX) 

add_executable(main_rrt_single "./main_rrt_single.cpp")
target_link_libraries(main_rrt_single Threads::Threads inverse_kinematics file_saver  parsing robot planner_RRT rapidxml obstacles DNN )
set_target_properties(main_rrt_single PROPERTIES LINKER_LANGUAGE CXX) 

if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()
