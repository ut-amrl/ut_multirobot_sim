function Vector2(x, y)
  return {x = x, y = y}
end

function Vector3(x, y, z)
  return {x, y, z}  -- Return array-style table for Eigen::Vector3f
end

function DegToRad(d)
  return math.pi * d / 180
end

-- UT JACKAL ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.43
car_length = 0.50
laser_loc = Vector3(0.07, 0, 0.5)

-- DIFFERENTIAL DRIVE MODEL PARAMETERS
invert_linear_vel_cmds = false
invert_angular_vel_cmds = false
linear_pos_accel_limit = 3.0  -- Forward acceleration limit [m/s²]
linear_neg_accel_limit = 3.0  -- Reverse acceleration limit [m/s²]
angular_pos_accel_limit = 3.0  -- Angular acceleration limit (CCW) [rad/s²]
angular_neg_accel_limit = 3.0  -- Angular acceleration limit (CW) [rad/s²]
max_angular = 3.0  -- Maximum angular velocity [rad/s]
max_linear_vel = 2.0  -- Maximum linear velocity [m/s]
