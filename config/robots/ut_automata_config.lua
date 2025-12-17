function Vector3(x, y, z)
  return {x, y, z}  -- Return array-style table for Eigen::Vector3f
end

function DegToRad(d)
  return math.pi * d / 180
end

-- UT AUTOMATA ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.281
car_length = 0.535
laser_loc = Vector3(0.2, 0.0, 0.15)

-- ACKERMANN DRIVE MODEL PARAMETERS
min_turn_radius = 0.98       -- Minimum turning radius [m]
max_speed = 1.2              -- Maximum velocity [m/s]
max_accel = 3.0              -- Maximum acceleration [m/s²]
turning_error_bias = DegToRad(0)   -- Systematic turning error [rad/s]
turning_error_rate = 0.1     -- Turning error per curvature [unitless]
