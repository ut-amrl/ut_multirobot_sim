function Vector3(x, y, z)
  return {x, y, z}
end

-- Laser pose relative to base_link
laser_loc = Vector3(0.2, 0.0, 0.0)

-- Omnidirectional drive parameters (NOTE: limits disabled)
apply_limits = false
max_speed = 1.2             -- Maximum linear velocity [m/s]
max_accel = 3.0             -- Maximum linear acceleration [m/s²]
max_angular_vel = math.pi   -- Maximum angular velocity [rad/s]
max_angular_accel = math.pi -- Maximum angular acceleration [rad/s²]
