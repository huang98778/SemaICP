cmake_minimum_required(VERSION 2.8)
project(sem_ICP)

if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native")
	message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(CMAKE_BUILD_TYPE  "Debug")



find_package(PCL   REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})



add_executable(icpVisu  main.cpp)
target_link_libraries( icpVisu  ${PCL_LIBRARIES})

                    


