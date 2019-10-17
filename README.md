# F1/10 Simulator

## Dependencies

1. glog
1. popt

You can install all the dependencies on *buntu using:
```
sudo apt install libgoogle-glog-dev libpopt-dev
```

Add the project directory to `ROS_PACKAGE_PATH`:
```
export ROS_PACKAGE_PATH=MYDIRECTORY:$ROS_PACKAGE_PATH
```
(Replace `MYDIRECTORY` with the actual directory)
You can also add this to your `~/.bashrc` file so that you don't have to do 
this every time you open a new terminal.


## Build


Build the program

`make`


## Run

Run

`./bin/cobot_simulator`

Publishes a laser scan message, odometry message, and visualization message
Markers and listens to a drive message.

Uses the first map in the file maps/atlas.txt.
