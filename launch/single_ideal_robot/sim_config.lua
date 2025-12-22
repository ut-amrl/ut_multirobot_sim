function Vector2(x, y)
  return {x = x, y = y}
end

function Vector3(x, y, z)
  return {x, y, z}  -- Return array-style table for Eigen::Vector3f
end

function DegToRad(d)
  return math.pi * d / 180
end

-- Map name (initial map before listening to current_map_topic)
map_name = "EmptyMap"

-- Current map topic for dynamic map switching
current_map_topic = "/current_map"

-- Simulation timestep
delta_t = 0.050

-- Laser scan ROS topics and frames
laser_topic = "scan"
laser_frame = "/base_laser"

-- Laser scan settings approximating Livox Mid-360
laser_noise_stddev = 0.02
laser_angle_min = DegToRad(-180.0)
laser_angle_max = DegToRad(180.0)
laser_angle_increment = DegToRad(0.2)
laser_min_range = 0.1
laser_max_range = 70.0

-- ROBOT FLEET
-- Each index corresponds to one robot (robot_types[i], start_poses[i], robot_configs[i])

robot_types = { "IDEAL_DRIVE" }
start_poses = { Vector3(0, 0, 0) }  -- x, y, theta_rad  
robot_configs = { "launch/single_ideal_robot/ideal_config.lua" }

-- Example: Add more robots by extending all three arrays
-- robot_types = { "DIFF_DRIVE", "ACKERMANN_DRIVE", "OMNIDIRECTIONAL_DRIVE" }
-- start_poses = { Vector3(0,0,0), Vector3(5,0,0), Vector3(10,0,0) }
-- robot_configs = {
--     "config/robots/ut_jackal_config.lua",
--     "config/robots/ut_automata_config.lua",
--     "config/robots/cobot_config.lua"
-- }

-- DYNAMIC OBJECTS (leave empty to disable entities/humans)
short_term_object_config_list = {}
human_config_list = {}
