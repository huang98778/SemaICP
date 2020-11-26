cmake_minimum_required(VERSION 2.8)
project(2dTree)

if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native")
	message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(CMAKE_BUILD_TYPE  "Release")

include_directories(  src)

set(Tree_SRC    src/2DTree.cpp )

         
add_library(${PROJECT_NAME} SHARED ${Tree_SRC})

# cmake test
add_executable(test  test/test.cpp)
target_link_libraries( test   ${PROJECT_NAME} )

