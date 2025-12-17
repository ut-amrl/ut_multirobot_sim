function Vector2(x, y)
  return {x = x, y = y}
end

function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

function DegToRad(d)
  return math.pi * d / 180
end

-- example of loading human crow scenario configs
-- init_config_file = "config/scenarios/human_crowd/example_scenario/init_config.lua"

-- ENVIRONMENT CONFIGURATION
-- Pure environment settings: map, simulation timestep, sensors, dynamic objects

-- Map name (without path/extension, e.g., "GDC1")
map_name = "GDC1"

-- Simulation timestep
delta_t = 0.025

-- Laser scan ROS topics and frames
laser_topic = "scan"
laser_frame = "base_laser"

-- Laser scan settings (sensor specs, same for all robots)
laser_noise_stddev = 0.01
laser_angle_min = DegToRad(-135.0)
laser_angle_max = DegToRad(135.0)
laser_angle_increment = DegToRad(0.25)
laser_min_range = 0.4
laser_max_range = 100.0

-- ROBOT FLEET CONFIGURATION
-- Define robots to simulate: parallel arrays of equal length
-- Each index corresponds to one robot (robot_types[i], start_poses[i], robot_configs[i])

robot_types = { "DIFF_DRIVE" }
start_poses = { Vector3(0, 0, 0) }  -- x, y, theta_rad  
robot_configs = { "config/robots/ut_jackal_config.lua" }

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
