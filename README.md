# UT Multi-Robot Simulator

ROS2 Jazzy fixed-timestep simulator for multiple robots (Ackermann, diff drive, omni) in 2D maps with optional dynamic objects.

## Dependencies
- ROS2: `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `ament_index_cpp`, `amrl_msgs`
- System: glog, gflags, Lua5.1, Eigen3  
Install system deps on Ubuntu:
```bash
sudo apt install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev libeigen3-dev
```

## Architecture (what runs)
Fixed-timestep ROS2 node that loads configs, builds robots, and steps physics.
```
┌─────────────────────────────────────────────────────────────────┐
│                        simulator_main.cpp                        │
│  (ROS2 node; parses flags; sim state pub/sub)                   │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Simulator Class                          │
│  Manages robots, environment, sensors, TF, dynamic objects.     │
└──────┬──────────────┬──────────────┬──────────────┬─────────────┘
       ▼              ▼              ▼              ▼
  Robot 1         Robot 2         Robot N       Environment
  (drive model)   (drive model)   (drive model) (map + objects)
```

### Core pieces
- `simulator_main.cpp`: ROS2 entry; publishes `/sim_state`; subscribes to `/sim_start_stop`, `/sim_step`; launches `Simulator`.
- `simulator.cpp`: loads map/config, creates motion models, loads humans/short-term objects, runs `Run()` each tick (update physics, publish odom/laser/viz/TF, optional localization).
- Drive models (`src/simulator/drive_models`):
  - Ackermann: `/{robot}/ackermann_drive` (`amrl_msgs/AckermannCurvatureDriveMsg`)
  - Diff drive: `/{robot}/cmd_vel` (`geometry_msgs/Twist`)
  - Omni: `/{robot}/cobot_drive` (`ut_multirobot_sim/CobotDriveMsg`)
- Entities (`src/simulator/entities`): humans (single-shot/repeat goals) and short-term obstacles share `EntityBase` geometry/pose.

### Simulation loop (flow)
```
INIT (once): read env/robot/init configs → load map → build motion models →
             load humans/short-term objects → setup pubs/subs → TF broadcaster

MAIN LOOP (fixed dt):
  spin_some()
  if RUNNING or (STOPPED + sim_step): simulator.Run()
  publish sim_state (step count, sim time)
  sleep to maintain dt
```
Robot step: drive timeout → accel limits → integrate pose (+ optional noise) → publish odom/laser/TF/viz/true pose (and localization if enabled).  
Dynamic objects: step toward goals → update collision lines → publish viz.  
Laser: cast rays from laser pose vs map + objects → add noise → publish `LaserScan`.

## Build
```bash
source /opt/ros/jazzy/setup.bash
git submodule update --init --recursive
colcon build --packages-select ut_multirobot_sim   # or: make
```

## Run
```bash
source install/setup.bash
ros2 run ut_multirobot_sim simulator \
  --env_config config/environment/sim_config.lua \
  --robot_config config/robots/ut_jackal_config.lua \
  --init_config config/initialization/default_init_config.lua \
  --maps_dir /path/to/amrl_maps \
  [--localize]   # publish /{robot}/localization
# or: ros2 launch ut_multirobot_sim ut_jackal.launch.py
```

## Configuration (what to edit)
- `config/environment/sim_config.lua`: timing, map, laser, TF flags, dynamic object defaults.
- `config/robots/*.lua`: motion model params and topics per robot type.
- `config/initialization/*.lua`: map name and start poses per robot.
- `config/dynamic_objects/...`: optional humans and short-term obstacles.
- Optional generated scenarios: `config/scenarios/human_crowd/<prefix>/init_config.lua` (via `scripts/generate_config.py`).

## Topics and TF
- Publishes per robot: `/odom`, `/scan` (or configured laser topic), `/simulator_true_pose`, `/simulator_visualization`, `/localization` (when `--localize`).
- Subscribes per robot: drive topic (per model above), `/initialpose`.
- Global: `/sim_state`, `/sim_start_stop`, `/sim_step`.
- TF (per robot): `map → odom → base_footprint → base_link → base_laser`.

## Navigation integration
- Ground truth: `/robot{N}/simulator_true_pose` always; `/robot{N}/localization` (with `--localize`) includes map name.
- Commands: diff drive `/cmd_vel`; Ackermann `/{robot}/ackermann_drive` (`amrl_msgs/AckermannCurvatureDriveMsg`, remap to `/ackermann_curvature_drive` if needed); omni `/{robot}/cobot_drive`.
- Run with localization (example above) and remap topics in your launch file as required.
- Geometry: supports `car_width`, `car_length`, `car_height`, `laser_loc`, `rear_axle_offset`; does not implement `base_link_offset`—navigation configs using it may see small offsets; tune dimensions/offsets accordingly.

## Multi-robot
- In init config, set `robot_types` and `start_poses` arrays of equal length; namespaces become `/robot0`, `/robot1`, ...
- Send commands to each namespace independently (e.g., `/robot1/cmd_vel`).

## Visualization
Use RViz2 (`ros2 run rviz2 rviz2`); fixed frame `map`; add `/simulator_visualization` and `/robot0/scan` (or your laser topic); view TF tree.

## Scenario generator (scripts/)
- Define a crowd scenario in `scripts/example.yml` (or your own YAML) and run:
```bash
python scripts/generate_config.py --config-file scripts/example.yml
```
Outputs under `config/scenarios/human_crowd/<prefix>/` (`init_config.lua` + per-human configs). Use the generated `init_config.lua` via `--init_config` or set `init_config_file` inside `config/environment/sim_config.lua`.

## Debugging / quick checks
- `ros2 topic list`, `ros2 topic echo /sim_state` to confirm the loop.
- `ros2 topic echo /robot0/odom` or `/robot0/scan` to verify motion/sensing.
- Empty scans or TF issues? Check laser params and TF flags (`publish_tfs`, `publish_map_to_odom`, `publish_foot_to_base`) in env/robot configs.
- Map load issues? Confirm `map_name` and `--maps_dir`.
