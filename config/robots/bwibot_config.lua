function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

-- BWIBOT ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.34
car_length = 0.34
car_height = 1.5
rear_axle_offset = 0.0
laser_loc = Vector3(0.15, 0, 0.155)

-- DIFFERENTIAL DRIVE MODEL PARAMETERS
invert_linear_vel_cmds = false
invert_angular_vel_cmds = false
linear_pos_accel_limit = 3.0
linear_neg_accel_limit = 3.0
angular_pos_accel_limit = 3.0
angular_neg_accel_limit = 3.0
max_angular = 3.0
max_linear_vel = 3.0
linear_odom_scale = 1.0
angular_odom_scale = 1.0