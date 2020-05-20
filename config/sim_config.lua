function Vector2(x, y)
  return {x = x, y = y}
end

function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

function DegToRad(d)
  return math.pi * d / 180
end

init_config_file = "config/default_init_config.lua"
-- example of loading human crow scenario configs
-- init_config_file = "config/human_crowd_scenario_configs/example_scenario/init_config.lua"

-- Time-step for simulation.
delta_t = 0.025

-- Simulator TF publications
publish_foot_to_base = true;
publish_map_to_odom = true;

-- Car dimensions.
car_width = 0.281
car_length = 0.535
car_height = 0.15;

-- Location of the robot's rear wheel axle relative to the center of the body.
rear_axle_offset = -0.162
laser_loc = Vector3(0.2, 0, 0.15)

-- Kinematic and dynamic constraints for the car.
min_turn_radius = 0.98
max_speed = 1.2
max_accel = 3.0

-- Laser noise simulation.
laser_noise_stddev = 0.01

-- Turning error simulation.
angular_error_bias = DegToRad(0);
angular_error_rate = 0.1;

-- Defining robot type enumerator
local RobotType = {
    F1TEN="F1TEN",
    COBOT="COBOT",
    BWIBOT="BWIBOT"
}

-- robot_type = RobotType.F1TEN
-- robot_config = "config/ackermann_config.lua"
-- robot_type = RobotType.BWIBOT
-- robot_config = "config/bwibot_config.lua"
robot_type = RobotType.COBOT
robot_config = "config/cobot_config.lua"

laser_topic = "/Cobot/Laser"
