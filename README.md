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
- `simulator.cpp`: loads map/config, creates motion models, loads humans/short-term objects, runs `Run()` each tick (update physics, publish odom/laser/TF/localization).
- Drive models (`src/simulator/drive_models`):
   - Ackermann: `/{robot}/cmd_vel` (`geometry_msgs/Twist` → interprets as velocity + angular velocity, converts to curvature internally)
   - Diff drive: `/{robot}/cmd_vel` (`geometry_msgs/Twist` → direct velocity control)
   - Omni: `/{robot}/cmd_vel` (`geometry_msgs/Twist` → independent x/y/rotation control)
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
Robot step: drive timeout → accel limits → integrate pose (+ optional noise) → publish odom/laser/TF/localization.  
Dynamic objects: step toward goals → update collision lines.  
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
ros2 run ut_multirobot_sim simulator --config config/environment/sim_config.lua

# Optional: Specify custom maps directory
ros2 run ut_multirobot_sim simulator \
  --config config/environment/sim_config.lua \
  --maps_dir /path/to/amrl_maps
```

## Launch Files

Single omni-drive stack (`launch/single_ideal_robot_launch.py`):
- Launches simulator + graph_navigation + webviz with proper topic remappings for `/robot0` namespace
```bash
ros2 launch ut_multirobot_sim single_ideal_robot_launch.py
```

## Configuration

**Config Structure:**
- `config/environment/sim_config.lua` - Main config containing:
  - Environment: map name, simulation timestep
  - Sensors: laser scan settings
  - Robot fleet: **parallel arrays** defining each robot:
    - `robot_types`: Drive model type per robot (DIFF_DRIVE, ACKERMANN_DRIVE, OMNIDIRECTIONAL_DRIVE)
    - `start_poses`: Initial pose per robot (x, y, theta)
    - `robot_configs`: Config file path per robot
  - Dynamic objects: humans and obstacles
- `config/robots/*.lua` - Per-robot configs containing:
  - Laser location (`laser_loc`)
  - Drive model parameters: speeds, accelerations, odometry scales, etc.

**Common Edits:**
- **Single robot**: Set all three arrays with one entry each in `sim_config.lua`
- **Multiple robots (different types)**: Extend all three arrays:
  ```lua
  robot_types = { "DIFF_DRIVE", "ACKERMANN_DRIVE" }
  start_poses = { Vector3(0,0,0), Vector3(5,0,0) }
  robot_configs = { "config/robots/ut_jackal_config.lua", "config/robots/ut_automata_config.lua" }
  ```
- **Disable entities**: Set `short_term_object_config_list = {}` and `human_config_list = {}` in `sim_config.lua`
- `config/dynamic_objects/...`: optional humans and short-term obstacles.

## Topics and TF
- Publishes per robot: `/odom`, `/scan` (or configured laser topic), `/localization` (ground truth with map).
- Subscribes per robot: `/{robot}/cmd_vel` (`geometry_msgs/Twist`), `/initialpose`.
- Global: `/sim_state`, `/sim_start_stop`, `/sim_step`.
- TF (per robot): `map → odom → base_link → base_laser`.

## Navigation integration
- Ground truth localization: `/robot{N}/localization` (`amrl_msgs/Localization2DMsg`) includes pose and map name.
- Commands: all robots use `/robot{N}/cmd_vel` (`geometry_msgs/Twist`).
- Odometry: `/robot{N}/odom` (`nav_msgs/Odometry`) with pose and twist covariance.
- Laser pose: Configurable via robot config: `laser_loc`.

## Multi-robot
- In `sim_config.lua`, set parallel arrays (`robot_types`, `start_poses`, `robot_configs`) of equal length
- Each robot gets its own namespace: `/robot0`, `/robot1`, ...
- Each robot can have a different type and config file:
  ```lua
  robot_types = { "DIFF_DRIVE", "ACKERMANN_DRIVE", "OMNIDIRECTIONAL_DRIVE" }
  start_poses = { Vector3(0,0,0), Vector3(5,0,0), Vector3(10,0,0) }
  robot_configs = {
      "config/robots/ut_jackal_config.lua",
      "config/robots/ut_automata_config.lua",
      "config/robots/cobot_config.lua"
  }
  ```
- Send commands to each namespace independently (e.g., `/robot1/cmd_vel`)

## Scenario generator (scripts/)
- Define a crowd scenario in `scripts/example.yml` (or your own YAML) and run:
```bash
python scripts/generate_config.py --config-file scripts/example.yml
```
Outputs scenario configs under `config/scenarios/human_crowd/<prefix>/`. Reference the generated configs in `sim_config.lua` by updating `human_config_list` and `start_poses`.

## Debugging / quick checks
- `ros2 topic list`, `ros2 topic echo /sim_state` to confirm the loop.
- `ros2 topic echo /robot0/odom` or `/robot0/scan` to verify motion/sensing.
- Empty scans or TF issues? Check laser params and ensure TF is being published.
- Map load issues? Confirm `map_name` and `--maps_dir`.
