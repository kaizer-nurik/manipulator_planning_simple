cmake_minimum_required(VERSION 3.10)
project(common_libraries_for_planners C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RELEASE_FLAGS "-O3")
set(DEBUG_FLAGS "-O3")

set(CMAKE_CXX_FLAGS_RELEASE ${RELEASE_FLAGS})
set(CMAKE_C_FLAGS_RELEASE ${RELEASE_FLAGS})

set(CMAKE_CXX_FLAGS_DEBUG ${DEBUG_FLAGS})
set(CMAKE_C_FLAGS_DEBUG ${DEBUG_FLAGS})

include_directories(../rapidxml ./include  )

set(LIB_SOURCE_FILES "./src/robot.cpp")
set(LIB_HEADERS_FILES "./include/robot.h")
add_library(robot STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(robot PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/obstacles.cpp")
set(LIB_HEADERS_FILES "./include/obstacles.h")
add_library(obstacles STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(obstacles PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/parsing.cpp")
set(LIB_HEADERS_FILES "./include/parsing.h")
add_library(parsing STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(parsing PROPERTIES LINKER_LANGUAGE CXX)

set(LIB_SOURCE_FILES "./src/inverse_kinematics.cpp")
set(LIB_HEADERS_FILES "./include/inverse_kinematics.h")
add_library(inverse_kinematics STATIC ${LIB_HEADERS_FILES} ${LIB_SOURCE_FILES}) 
set_target_properties(inverse_kinematics PROPERTIES LINKER_LANGUAGE CXX)

if(ENABLE_GCOV)
SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
endif()
