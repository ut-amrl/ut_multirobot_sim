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
delta_t = 0.02
command_timeout = 0.4

-- Laser
laser_topic = "scan"
laser_frame = "base_laser"
laser_noise_stddev = 0.04
laser_angle_min = DegToRad(-135.0)
laser_angle_max = DegToRad(135.0)
laser_angle_increment = DegToRad(0.2)
laser_min_range = 0.3
laser_max_range = 70.0

-- Robots
robot_types = { "OMNIDIRECTIONAL_DRIVE" }
start_poses = { Vector3(0, 0, 0) }
robot_configs = { "launch/single_ideal_robot/cobot_config.lua" }

-- Dynamic objects
short_term_object_config_list = {}
human_config_list = {}
