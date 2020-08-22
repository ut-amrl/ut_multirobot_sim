function Vector2(x, y)
    return {x = x, y = y}
end

-- Human shape information
hu_radius = 0.1
hu_num_segments = 20

-- Human start position
--hu_start_x = 33.
--hu_start_y = 21.5
--hu_start_theta = 0.

hu_waypoints = {
    {37, 20, 0},
    {30, 16, 0},
    {29, 9, 0},
    {15, 8.5, 0},
    {15, 15, 0}
}

-- Human goal position
--hu_goal_x = 42.
--hu_goal_y = 22.7
--hu_goal_theta = 0.

-- Human speed information
hu_max_speed = 1.
hu_avg_speed = 0.8
hu_max_omega = 0.2
hu_avg_omega = 0.
hu_reach_goal_threshold = 0.3

-- Human walking mode
local HumanMode = {
    Singleshot=0,
    Repeat=1,
    Controlled=2,
    Cycle=3
}

hu_mode = HumanMode.Singleshot
-- hu_control_topic = "/human1/command"
