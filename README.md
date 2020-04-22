# F1/10 Simulator [![Build Status](https://travis-ci.com/ut-amrl/f1tenth_simulator.svg?branch=master)](https://travis-ci.com/ut-amrl/f1tenth_simulator)

## Dependencies

1. [glog](https://github.com/google/glog)
1. [popt](https://directory.fsf.org/wiki/Popt)

You can install all the dependencies on *buntu using:
```
sudo apt install libgoogle-glog-dev libpopt-dev
```


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
1. Do **not** run `cmake`, `catkin_make`, `rosbuild` or any other such alternate commands that the internet may suggest.


## Run

Run `./bin/simulator`

The simulator laser scans to the `/laser` topic, odometry messages to `/odom`,
and visualization messages to `/simulator_visualization`. It listens to motion
commands on `/ackermann_drive`, and location initialization messages on
`/initialpose`.

## Visualize Simulation

Run `rosrun rviz rviz -d visualization.rviz`

