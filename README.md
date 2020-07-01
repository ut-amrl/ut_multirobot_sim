# UT Multi-Robot Simulator

![Build Status](https://travis-ci.com/ut-amrl/ut_multirobot_sim.svg?branch=master)

## Dependencies

1. [glog](https://github.com/google/glog)
1. [gflags](https://github.com/gflags/gflags)
1. [Lua5.1](http://www.lua.org/)

You can install all the dependencies on *buntu using:
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

