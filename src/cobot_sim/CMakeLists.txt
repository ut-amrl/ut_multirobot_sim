SET(target cobot_simulator)

ROSBUILD_ADD_BOOST_DIRECTORIES()

ROSBUILD_ADD_EXECUTABLE(${target}
  cobot_sim_main.cpp
  cobot_sim.cpp
)

ADD_DEPENDENCIES(${target}
  shared_libraries
  map_libraries
  rosbuild_precompile
)

TARGET_LINK_LIBRARIES(${target}
  shared_libraries
  map_libraries
  ${libs}
)

ROSBUILD_LINK_BOOST(${target} system)
