# UT Multi-Robot Simulator

Multi-robot simulator for ROS2 Jazzy.

## Dependencies

### ROS2 Packages
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `ament_index_cpp`
- `amrl_msgs` (custom AMRL message package)

### System Libraries
- [glog](https://github.com/google/glog)
- [gflags](https://github.com/gflags/gflags)
- [Lua5.1](http://www.lua.org/)
- [Eigen3](https://eigen.tuxfamily.org/)

Install system dependencies on Ubuntu:
```bash
sudo apt install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev libeigen3-dev
```

## Build

1. Ensure ROS2 Jazzy is sourced:
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

2. Pull all submodules:
    ```bash
    git submodule update --init --recursive
    ```

3. Build with colcon:
    ```bash
    colcon build --packages-select ut_multirobot_sim
    ```

   Or use the included Makefile:
    ```bash
    make
    ```

## Run

1. Source the install directory:
    ```bash
    source install/setup.bash
    ```

2. Run the simulator directly:
    ```bash
    ros2 run ut_multirobot_sim simulator \
      --env_config $(ros2 pkg prefix --share ut_multirobot_sim)/config/sim_config.lua \
      --robot_config $(ros2 pkg prefix --share ut_multirobot_sim)/config/ut_jackal_config.lua \
      --init_config $(ros2 pkg prefix --share ut_multirobot_sim)/config/default_init_config.lua
    ```

3. Or use the launch file:
    ```bash
    ros2 launch ut_multirobot_sim ut_jackal.launch.py
    ```

## Topics

The simulator publishes:
- `/sim_state` - Simulator state (running/stopped)
- `/robot0/odom` - Robot odometry
- `/robot0/scan` - Laser scan data
- `/robot0/simulator_true_pose` - Ground truth pose
- `/robot0/simulator_visualization` - Visualization markers
- `/simulator_visualization` - Map and object visualization

The simulator subscribes to:
- `/robot0/ackermann_drive` (or robot-specific drive topic) - Motion commands
- `/robot0/initialpose` - Initial localization
- `/sim_start_stop` - Start/stop simulation
- `/sim_step` - Step simulation

## Visualization

Visualize in RViz2:
```bash
ros2 run rviz2 rviz2
```

Add displays for:
- `/robot0/scan` (LaserScan)
- `/simulator_visualization` (Marker)
- TF tree

## Configuration

Robot and environment configurations are stored in `config/`:
- `sim_config.lua` - Environment and map settings
- `ut_jackal_config.lua` - UT Jackal robot parameters
- `bwibot_config.lua` - BWIBot robot parameters
- `cobot_config.lua` - Cobot robot parameters
- `default_init_config.lua` - Initial robot poses

## Multi-Robot Support

The simulator supports multiple robots. Configure robot types and start poses in `default_init_config.lua`:
```lua
robot_types = {"ACKERMANN_DRIVE", "DIFF_DRIVE"};
start_poses = {{0, 0, 0}, {2, 2, 1.57}};
```

## Maps

Maps are loaded from the `amrl_maps` package. Specify the map name in the init config or pass via `--maps_dir` flag.
