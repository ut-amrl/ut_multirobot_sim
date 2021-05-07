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
        { 36.04, 9.06, 13 }
    }

    hu1_waypoints = {
        { -10.06, 6.36, 7 }
    }

    hu2_waypoints = {
        { 13.72, 6.68, 6 }
    }

    hu3_waypoints = {
        { 0.74, 10.35, 7 }
    }

    hu4_waypoints = {
        { 0.74, 10.35, 20 }
    }

    hu5_waypoints = {
        { 21.29, 6.68, 14 }
    }

    hu6_waypoints = {
        { 13.72, 6.68, 19 }
    }

    hu7_waypoints = {
        { 36.04, 9.06, 22 }
    }

    hu8_waypoints = {
        { 9.9, 10.35, 19 }
    }

    hu9_waypoints = {
        { -36.1, 20.65, 21 }
    }

    hu10_waypoints = {
        { -6.38, 6.38, 1 }
    }

    hu11_waypoints = {
        { -14.38, 10.35, 19 }
    }

    hu12_waypoints = {
        { -12.02, 10.35, 31 }
    }

    hu13_waypoints = {
        { -24.39, 16.1, 5 }
    }

    hu14_waypoints = {
        { 6.01, 6.68, 13 }
    }

    hu15_waypoints = {
        { -19, 6.36, 12 }
    }

    hu16_waypoints = {
        { -10.06, 6.36, 16 }
    }

    hu17_waypoints = {
        { 14.03, 24.11, 3 }
    }
