function Vector3(x, y, z)
  return {x, y, z}
end

-- Laser pose relative to base_link
laser_loc = Vector3(0.15, 0, 0.155)

-- DIFFERENTIAL DRIVE MODEL PARAMETERS
invert_linear_vel_cmds = false
invert_angular_vel_cmds = false
linear_pos_accel_limit = 3.0  -- Forward acceleration limit [m/s²]
linear_neg_accel_limit = 3.0  -- Reverse acceleration limit [m/s²]
angular_pos_accel_limit = 3.0  -- Angular acceleration limit (CCW) [rad/s²]
angular_neg_accel_limit = 3.0  -- Angular acceleration limit (CW) [rad/s²]
max_angular = 3.0  -- Maximum angular velocity [rad/s]
max_linear_vel = 3.0  -- Maximum linear velocity [m/s]
