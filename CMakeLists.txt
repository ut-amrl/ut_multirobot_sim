PROJECT(f1tenth_simulator)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Wall -g -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -rdynamic")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

INCLUDE(FindLuaJIT)
FIND_PACKAGE(LuaJIT REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/map)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared/math)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared/util)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader/include)
INCLUDE_DIRECTORIES(${LUAJIT_INCLUDE_DIR})


ROSBUILD_GENMSG()

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/cobot_sim)

SET(libs roslib roscpp popt glog
    ${BUILD_SPECIFIC_LIBRARIES} rosbag X11)

SET(SHARED_SRCS
  src/shared/math/geometry.h
  src/shared/util/helpers.cpp
  src/shared/util/proghelp.cc
  src/shared/util/terminal_utils.cpp
  src/shared/util/watch_files.cpp
)

SET(MAP_SRCS
  src/map/vector_map.cpp
)

#==============================================================================
# Creating libraries to eliminate redundant compile steps
#
ADD_LIBRARY(shared_libraries SHARED ${SHARED_SRCS} ${HW_SRCS})
ADD_DEPENDENCIES(shared_libraries rosbuild_precompile)
TARGET_LINK_LIBRARIES(shared_libraries shared_libraries ${LUAJIT_LIBRARIES} ${libs})

ADD_LIBRARY(map_libraries SHARED ${MAP_SRCS})
ADD_DEPENDENCIES(map_libraries shared_libraries)
TARGET_LINK_LIBRARIES(map_libraries shared_libraries ${libs})