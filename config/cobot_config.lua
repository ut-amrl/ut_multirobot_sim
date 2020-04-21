function DegToRad(d)
  return math.pi * d / 180
end

co_drive_callback_topic = "/Cobot/Drive"
co_cobot_odom_topic = "/Cobot/Odometry"
co_base_radius = 0.2
co_w0 = DegToRad(45.0)
co_w1 = DegToRad(135.0)
co_w2 = DegToRad(-135.0)
co_w3 = DegToRad(-45.0)

co_max_angle_vel = math.pi
co_max_angle_accel = math.pi
-- Kinematic and dynamic constraints
co_min_turn_radius = 0.98
co_max_speed = 1.2
co_max_accel = 3.0
