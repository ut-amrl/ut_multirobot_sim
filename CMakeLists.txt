cmake_minimum_required(VERSION 3.8)
project(ut_multirobot_sim)

MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# C++17 required for ROS2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -Werror -g")

if(CMAKE_BUILD_TYPE MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -fopenmp")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

# Define ROS2 for compatibility with shared libraries
add_compile_definitions(ROS2)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Handle AMENT_PREFIX_PATH for custom packages
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" ament_prefix_path_list "$ENV{AMENT_PREFIX_PATH}")
  foreach(path ${ament_prefix_path_list})
    list(APPEND CMAKE_PREFIX_PATH "${path}")
  endforeach()
endif()

find_package(amrl_msgs REQUIRED)

# System libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(LUA REQUIRED lua5.1)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/submodules
  ${PROJECT_SOURCE_DIR}/submodules/config_reader
  ${PROJECT_SOURCE_DIR}/submodules/config_reader/include
  ${PROJECT_SOURCE_DIR}/submodules/shared
  ${LUA_INCLUDE_DIRS}
)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimulatorStateMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs amrl_msgs
)

# Build shared library submodule
add_subdirectory(submodules/shared)

# Simulator executable
add_executable(simulator
  src/simulator/simulator_main.cpp
  src/simulator/simulator.cpp
  src/simulator/vector_map.cpp
  src/simulator/entities/entity_base.cpp
  src/simulator/drive_models/robot_model.cpp
  src/simulator/drive_models/ackermann_model.cpp
  src/simulator/drive_models/omnidirectional_model.cpp
  src/simulator/drive_models/diff_drive_model.cpp
  src/simulator/entities/short_term_object.cpp
  src/simulator/entities/human_object.cpp
)

# Link ROS2 dependencies
ament_target_dependencies(simulator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  ament_index_cpp
  amrl_msgs
)

# Link to generated messages (same package) - compatible with Foxy/Humble/Jazzy
if(${rosidl_cmake_VERSION} VERSION_LESS "3.1.5")
  # Foxy approach
  rosidl_target_interfaces(simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  # Humble/Jazzy approach
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(simulator "${cpp_typesupport_target}")
endif()

# Link system libraries
target_link_libraries(simulator
  amrl_shared_lib
  glog
  gflags
  ${LUA_LIBRARIES}
)

# Install executable
install(TARGETS simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  amrl_msgs
  rosidl_default_runtime
)

ament_package()
