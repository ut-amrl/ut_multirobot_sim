require("config.sim_config");
function DegToRad(d)
  return math.pi * d / 180
end

-- Kinematic and dynamic constraints for the car.
ak_min_turn_radius = 0.98
ak_max_speed = 5.0
ak_max_accel = 3.0

-- Turning error simulation.
ak_angular_error_bias = DegToRad(0);
ak_angular_error_rate = 0.1;

ak_drive_callback_topic = "/ackermann_curvature_drive"

-- Car configuration
ackermann_num_segments = 20
ackermann_radius = 0.05
ackermann_offset = {0.02, 0} -- position of lidar
