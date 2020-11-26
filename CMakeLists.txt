cmake_minimum_required(VERSION 2.8)
project(sem_ICP)

if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native")
	message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(sem_ICP_VERSION_MAJOR 1)
set(sem_ICP_VERSION_MINOR 0)  ## version info

set(CMAKE_BUILD_TYPE  "Debug")

include_directories("/usr/include/eigen3" "../common")

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(submodules/utility)
include_directories(submodules/utility/include)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# find_package(OpenCV 3.4 REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
add_subdirectory(externals/nlohmann_json)
include_directories(externals/nlohmann_json/include)

find_package(PCL   REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_subdirectory(externals/sophus)
include_directories(${SOPHUS_INCLUDE_DIRS})

# find_package(Sophus REQUIRED)
# include_directories(${Sophus_INCLUDE_DIRS})

# boost 1.58
find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)

# file(GLOB srcs *.c*)
# file(GLOB headers *.h*)

# include_directories( header )
include_directories(
    src/include
    externals/sophus/
)

set(GICP_SRC    #main.cpp                
                src/src/PerceptionParser.cpp
                src/src/EMICP.cpp
                ${Boost_INCLUDE_DIRS}
                src/src/SemiICP.cpp 
                src/src/SemanticPointCloud.cpp
                src/src/PointCloudAdapter.cpp
                )

# set(MAIN_SRC mapprojICP.cpp
#             ${FUN_SRC})                
add_library(ICPLib SHARED ${GICP_SRC})

add_executable(semiICP  src/mapprojICP.cpp)
target_link_libraries( semiICP   #${PROJECT_NAME}                            
                                    ICPLib
                                    nlohmann_json::nlohmann_json
                                    ${Boost_LIBRARIES}
                                    ${CERES_LIBRARIES}
                                    ${PCL_LIBRARIES}
                                    glog
                                    utility
                                    sophus
                                    )

                    


