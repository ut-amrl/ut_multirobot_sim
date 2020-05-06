-- define multi robot senerio
robot_number = 2

-- topic prefix of robots
topic_prefix_list = {"car0", "car1", "car2"}

-- Defining robot type enumerator
local RobotType = {
    F1TEN=0,
    COBOT=1
}

-- type of robot
type_list = {0, 0, 0}

-- init cartesian location of robots
location_list = {{0.0, 0.0}, {-10.0, 10.0}, {10.0, 10.0}}

-- init angle of robots
angle_list = {0.0, 0.0, 0.0}

-- color of robots
RGB_list = {{255.0, 165.0, 39.0}, {52.0, 101.0, 164.0}, {92.0, 53.0, 102.0}}
