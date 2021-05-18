function Vector2(x, y)
    return {x = x, y = y}
end

-- Human shape information
hu_radius = 0.2
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
hu_control_topic = "/command"


    hu0_waypoints = {
        { 36.124, 15.972, 29 }
    }

    hu1_waypoints = {
        { -36.1, 18.18, 2 }
    }

    hu2_waypoints = {
        { 23.49, 6.68, 10 }
    }

    hu3_waypoints = {
        { -13.71, 16.24, 0 }
    }

    hu4_waypoints = {
        { 34.3, 7.04, 13 }
    }

    hu5_waypoints = {
        { -24.39, 16.1, 29 }
    }

    hu6_waypoints = {
        { 14.03, 24.11, 22 }
    }

    hu7_waypoints = {
        { 15.09, 10.35, 14 }
    }

    hu8_waypoints = {
        { -1.6, 16.24, 1 }
    }
