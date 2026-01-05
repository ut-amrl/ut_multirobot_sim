function Vector3(x, y, z)
  return {x, y, z}
end

function DegToRad(d)
  return math.pi * d / 180
end

-- Map
map_name = "EmptyMap"
current_map_topic = "/current_map"

-- Timing
delta_t = 0.025
command_timeout = 0.1

-- Laser
laser_topic = "scan"
laser_frame = "base_laser"
laser_noise_stddev = 0.01
laser_angle_min = DegToRad(-135.0)
laser_angle_max = DegToRad(135.0)
laser_angle_increment = DegToRad(0.25)
laser_min_range = 0.4
laser_max_range = 100.0

-- Robots
robot_types = { "OMNIDIRECTIONAL_DRIVE" }
start_poses = { Vector3(0, 0, 0) }
robot_configs = { "config/robots/cobot_config.lua" }

-- Dynamic objects
short_term_object_config_list = {}
human_config_list = {}
