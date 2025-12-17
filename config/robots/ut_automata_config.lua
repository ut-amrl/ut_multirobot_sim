require("config.environment.sim_config");

-- Kinematic and dynamic constraints for the car.
ak_min_turn_radius = 0.98
ak_max_speed = 1.2
ak_max_accel = 3.0

-- Turning error simulation.
ak_angular_error_bias = DegToRad(0);
ak_angular_error_rate = 0.1;

-- Drive topic standardized to "/cmd_vel" in simulator
