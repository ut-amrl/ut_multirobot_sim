cmake_minimum_required(VERSION 3.16.3)
project(state_switcher_rviz_plugin)

# Load rosbuild CMake stuff
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

set(ROS_BUILD_SHARED_LIBS true)

# Find Qt
find_package(Qt5 COMPONENTS Widgets)
if(Qt5_FOUND)
    message("Using Qt 5")
    set(CMAKE_AUTOMOC ON)
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
else()
    message("Using Qt 4")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    qt_wrap_cpp(MOC_FILES src/state_switcher.hpp)
    include(${QT_USE_FILE})
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCES src/state_switcher.cpp ${MOC_FILES})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(state_switcher_rviz_plugin
    ${SOURCES}
    src/state_switcher.hpp
    ${MOC_FILES})
target_link_libraries(state_switcher_rviz_plugin ${QT_LIBRARIES})