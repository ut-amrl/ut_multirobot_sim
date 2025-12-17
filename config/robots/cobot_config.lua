function Vector3(x, y, z)
  return {x, y, z}  -- Return array-style table for Eigen::Vector3f
end

function DegToRad(d)
  return math.pi * d / 180
end

-- COBOT ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.4
car_length = 0.4
laser_loc = Vector3(0.0, 0.0, 0.15)

-- OMNIDIRECTIONAL DRIVE MODEL PARAMETERS
max_speed = 1.2             -- Maximum linear velocity [m/s]
max_accel = 3.0             -- Maximum linear acceleration [m/s²]
max_angular_vel = math.pi   -- Maximum angular velocity [rad/s]
max_angular_accel = math.pi -- Maximum angular acceleration [rad/s²]
