# UT Multi-Robot Simulator

![Build Status](https://travis-ci.com/ut-amrl/ut_multirobot_sim.svg?branch=master)

## Dependencies

1. [glog](https://github.com/google/glog)
1. [gflags](https://github.com/gflags/gflags)
1. [Lua5.1](http://www.lua.org/)
1. [Robot Operating System](http://wiki.ros.org/ROS/Installation)
1. [Pedsim ROS](https://github.com/ut-amrl/pedsim_ros/tree/utmrs-integration) - Using the specific fork and branch provided here.

You can install some of these dependencies on *buntu using:
```
sudo apt install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev
```

To set up the pre-commit and pre-push hooks that verify changes are able to build locally, run
```
scripts/setup_hooks.sh
```
from inside the root of the repo.

## Build

1. Add the project directory to `ROS_PACKAGE_PATH`:
    ```
    export ROS_PACKAGE_PATH=MYDIRECTORY:$ROS_PACKAGE_PATH
    ```
    (Replace `MYDIRECTORY` with the actual directory)
    You can also add this to your `~/.bashrc` file so that you don't have to do
    this every time you open a new terminal.
1. Build the program:
    ```
    make
    ```
    Optionally, to compile on all cores (make sure you have sufficient RAM!)
    ```
    make -j
    ```
1. Do **not** run `cmake`, `catkin_make`, `rosbuild`.


## Run

Run `./bin/simulator`

The simulator laser scans to the `/laser` topic, odometry messages to `/odom`,
and visualization messages to `/simulator_visualization`. It listens to motion
commands on `/ackermann_drive`, and location initialization messages on
`/initialpose`.

## Visualize Simulation

Run `rosrun rviz rviz -d visualization.rviz`

## Configuring and Running Social Navigation Scenarios
UTMRS currently provides two possible control methods for humans, and corresponding configuration setups and requirements. The first is a simple waypoint following model for humans that treats humans as dynamic obstacles that are unaware of other objects in the environment (including the robot and other humans). The second uses a fork of a ros pedestrian simulator (pedsim_ros) built on libpedsim to model human behavior based on the social force model. 

### Simple Human Model
An example of running UTMRS with the simple human model and all related configuration files can be found in 
`config/human_examples` and can be launched with
`roslaunch config/human_examples/human_example.launch`

Important configuration files for this example are:
1. `sim_config.lua` : The primary UTMRS configuration file that specifies other files as necessary
2. `ut_automata_config.lua` : The robot config file that specifies properties of the robot used in the example.
3. `humans.lua` : The human config file. All non-waypoint parameters are shared by all humans at this time. For each desired human there is one list of waypoints that describe the humans behavior. IMPORTANT: The naming scheme for waypoint lists is important. Each list should take the form huI_waypoints, where I is the index of the human starting from 0.

### Social Force Human Model
An example of running UTMRS with the pedsim_ros based social force human model and all related configuration files can be found in 
`config/pedsim_example` and can be launched with
`roslaunch config/pedsim_example/pedsim_example.launch`

Important configuration files for this example are:
1. `sim_config.lua` : The primary UTMRS configuration file that specifies other files as necessary
2. `jackal_config.lua` : The robot config file that specifies properties of the robot used in the example.
3. `humans.lua` : The human config file. All non-waypoint parameters are shared by all humans at this time. For each desired human there is one list of waypoints with a single entry that represents the starting location of the corresponding human. IMPORTANT: The naming scheme for waypoint lists is important. Each list should take the form `huI_waypoints`, where `I` is the index of the human starting from 0.
4. `scene.xml` : Translates the map, robot location, and human waypoints into a representation useable by pedsim_ros. Configuration of the environment and scene setup are in this file.
5. `pedsim.launch` : Launch file that calls pedsim_ros as necessary. Includes some pedsim configuration and simulation settings. Additional pedsim configuration is currently handled within pedsim_ros itself, and must be modified to alter human social force parameters.

