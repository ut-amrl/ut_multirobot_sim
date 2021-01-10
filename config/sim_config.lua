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
delta_t = 0.05

-- Simulator TF publications
publish_tfs = true;
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
max_speed = 5.0
max_accel = 5.0

-- Laser noise simulation.
laser_noise_stddev = 0.01

-- Turning error simulation.
angular_error_bias = DegToRad(0);
angular_error_rate = 0.1;

laser_topic = "/Cobot/Laser"
laser_frame = "base_laser"

-- define multi robot senerio
-- The simulator will only use the first robot_number items in each list
-- to configure each robot.
robot_number = 2

-- topic prefix of robots
topic_prefix_list = {"robot_0", "robot_1", "robot_2", "robot_3", "robot_4", "robot_5", "robot_6", "robot_7"}

-- Defining robot type enumerator
local RobotType = {
    ACKERMANN_DRIVE="ACKERMANN_DRIVE",
    OMNIDIRECTIONAL_DRIVE="OMNIDIRECTIONAL_DRIVE",
    DIFF_DRIVE="DIFF_DRIVE"
}

--(Tongrui Li: rplease use type_list and config_list below)
-- robot_type = RobotType.ACKERMANN_DRIVE
-- robot_config = "config/ut_automata_config.lua"
-- robot_type = RobotType.DIFF_DRIVE
-- robot_config = "config/bwibot_config.lua"
-- robot_type = RobotType.OMNIDIRECTIONAL_DRIVE
-- robot_config = "config/cobot_config.lua"
-- robot_type = RobotType.DIFF_DRIVE
-- robot_config = "config/ut_jackal_config.lua

-- type of robot
type_list = {RobotType.ACKERMANN_DRIVE, RobotType.ACKERMANN_DRIVE, RobotType.ACKERMANN_DRIVE, RobotType.DIFF_DRIVE, RobotType.OMNIDIRECTIONAL_DRIVE, RobotType.DIFF_DRIVE}

-- config of robot
config_list = {"config/ut_automata_config.lua", "config/ut_automata_config.lua", "config/ut_automata_config.lua", "config/bwibot_config.lua", "config/cobot_config.lua", "config/ut_jackal_config.lua"}

-- init cartesian location of robots
location_list = {{0.0, 0.0}, {0.0, 0.0}, {3.0, 0}}

-- init angle of robots
angle_list = {0.0, 0.0, 0.0}

-- color of robots
RGB_list = {{255.0, 165.0, 39.0}, {52.0, 101.0, 164.0}, {92.0, 53.0, 102.0}}

