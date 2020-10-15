PROJECT(ut_multirobot_sim)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

string(ASCII 27 ESCAPE)
IF(NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  MESSAGE(AUTHOR_WARNING
    "${ESCAPE}[1;33m"
    "vector_map.cpp contains a struct not using EIGEN_MAKE_ALIGNED_OPERATOR_NEW"
    " this may cause issues on your architecture."
    " See: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers"
    ".html"
    "${ESCAPE}[0m")
ENDIF()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")

IF(${CXX} MATCHES "g++")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
ENDIF()

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
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

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader/include)

ROSBUILD_GENMSG()

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/submodules/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/shared)

SET(libs roslib roscpp glog gflags amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES} rosbag X11 lua5.1 boost_system)

SET(target simulator)
ROSBUILD_ADD_EXECUTABLE(${target}
  src/simulator/simulator_main.cpp
  src/simulator/simulator.cpp
  src/simulator/vector_map.cpp
  src/simulator/entity_base.cpp
  src/simulator/robot_model.cpp
  src/simulator/ackermann_model.cpp
  src/simulator/omnidirectional_model.cpp
  src/simulator/diff_drive_model.cpp
  src/simulator/short_term_object.cpp
  src/simulator/human_object.cpp
  )
TARGET_LINK_LIBRARIES(${target}
  ${libs}
)

