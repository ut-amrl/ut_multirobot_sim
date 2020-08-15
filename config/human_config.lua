-- Human shape information
hu_radius = 0.1
hu_num_segments = 20

hu_waypoints = {
    -- x, y, theta
    {39, 18.7, 0.0},
    {42, 22.7, 0.0},
}

-- Human speed information
hu_max_speed = 1.5
hu_avg_speed = 1.
hu_max_omega = 0.2
hu_avg_omega = 0.
hu_reach_goal_threshold = 0.3

-- Human walking mode
local HumanMode = {
    Singleshot=0,
    Repeat=1,
    Controlled=2,
    Cycle=3,
}

hu_mode = HumanMode.Repeat
-- hu_control_topic = "/human1/command"