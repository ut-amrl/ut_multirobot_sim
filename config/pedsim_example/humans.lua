function Vector2(x, y)
    return {x = x, y = y}
end

-- Human shape information
hu_radius = 0.6
hu_num_segments = 20

-- Human speed information
hu_max_speed = 0.8 --
hu_avg_speed = 0.8 --
hu_max_omega = 0.2
hu_avg_omega = 0.
hu_reach_goal_threshold = 0.1

-- Human walking mode
local HumanMode = {
    Singleshot=0,
    Repeat=1,
    Controlled=2,
    Cycle=3,
}

hu_mode = HumanMode.Controlled
-- hu_control_topic = "/human1/command"
hu_control_topic = "/command"

hu0_waypoints = {
    { 11.517275250089156, 1.5, 0 },
}

hu1_waypoints = {
    { 20.095816727556944, 1.5, 0 },
}

hu2_waypoints = {
    { 31.672941367084128, 1.5, 0 },
}

hu3_waypoints = {
    { 22.869150982753368, 1.5, 0 },
}

hu4_waypoints = {
    { 29.92911989439247, 1.5, 0 },
}

hu5_waypoints = {
    { 22.244815464316222, 3.0, 0 },
}
