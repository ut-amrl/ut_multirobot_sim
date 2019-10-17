SET(HW_SRCS
  serial.cc
)

SET(DRIVE1_SRCS
  drive.cpp
  fcs.cpp
)

SET(DRIVE2_SRCS
  drive.cpp
  fcs.cpp
)

SET(DRIVE3_SRCS
  drive.cpp
  serial.cc
  fcs.cpp
  companion/companion_drive/amcparse.cpp
  companion/companion_drive/packet.cpp
  companion/companion_drive/packet_helpers.cpp
  companion/companion_drive/time_help.cpp
  companion/companion_drive/companion_drive.cpp
)

SET(LASER_SRCS
  laser_scanner.cpp
)

SET(JOYSTICK_SRCS
  joystick.cc
)


SET(target auto_start)
ROSBUILD_ADD_EXECUTABLE(${target} auto_start.cpp)
ADD_DEPENDENCIES(${target} shared_libraries)
TARGET_LINK_LIBRARIES(${target} shared_libraries)

# SET(target cobot1_drive)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     cobot1_drive/cobot_drive_main.cpp 
#     ${DRIVE1_SRCS})
# ADD_DEPENDENCIES(${target} shared_libraries)
# TARGET_LINK_LIBRARIES(${target} shared_libraries)

# SET(target cobot2_drive)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     cobot2_drive/cobot_drive_main.cpp
#     ${DRIVE2_SRCS})
# ADD_DEPENDENCIES(${target} shared_libraries)
# TARGET_LINK_LIBRARIES(${target} shared_libraries)

SET(target cobot3_drive)
ROSBUILD_ADD_EXECUTABLE(${target}
    cobot3_drive/cobot_drive_main.cpp
    ${DRIVE2_SRCS})
ADD_DEPENDENCIES(${target} shared_libraries)
TARGET_LINK_LIBRARIES(${target} shared_libraries)

# SET(target companion_drive)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     companion/companion_drive/companion_drive_main.cpp
#     ${DRIVE3_SRCS})
# ADD_DEPENDENCIES(${target} shared_libraries)
# TARGET_LINK_LIBRARIES(${target} shared_libraries)

SET(target cobot_laser)
ROSBUILD_ADD_EXECUTABLE(${target}
    laser_scanner_main.cpp 
    ${LASER_SRCS})
ADD_DEPENDENCIES(${target} shared_libraries)
TARGET_LINK_LIBRARIES(${target} shared_libraries)

# Companion laser
# SET(target companion_laser)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     companion/companion_laser_scanner_main.cpp
#     ${LASER_SRCS})
# ADD_DEPENDENCIES(${target} shared_libraries)
# TARGET_LINK_LIBRARIES(${target} shared_libraries)

# SET(target cobot_star_gazer)
# ROSBUILD_ADD_EXECUTABLE(${target} 
#     star_gazer_node.cpp
#     ${MAP_SRCS})
# ADD_DEPENDENCIES(${target}
#     shared_libraries
#     ${libs}
#     stargazer)
# TARGET_LINK_LIBRARIES(${target}
#     shared_libraries
#     ${libs}
#     stargazer
#     re2)

SET(target cobot_joystick)
ROSBUILD_ADD_EXECUTABLE(${target}
    cobot_joystick_main.cpp
    ${JOYSTICK_SRCS})
ADD_DEPENDENCIES(${target} shared_libraries)
TARGET_LINK_LIBRARIES(${target} shared_libraries)

SET(target crebot_drive)
ROSBUILD_ADD_EXECUTABLE(${target} drive_crebot.cpp)
ADD_DEPENDENCIES(${target}
    shared_libraries
    rosbuild_precompile)
TARGET_LINK_LIBRARIES(${target}
    shared_libraries
    ${libs})

SET(target odometry_calibration)
ROSBUILD_ADD_EXECUTABLE(${target} odometry_calibration_main.cpp)
ADD_DEPENDENCIES(${target} shared_libraries)
TARGET_LINK_LIBRARIES(${target} shared_libraries)

