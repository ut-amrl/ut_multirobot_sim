function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

function DegToRad(d)
  return math.pi * d / 180
end

-- UT AUTOMATA ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.281
car_length = 0.535
car_height = 0.15
rear_axle_offset = -0.162
laser_loc = Vector3(0.2, 0.0, 0.15)

-- ACKERMANN DRIVE MODEL PARAMETERS
ak_min_turn_radius = 0.98
ak_max_speed = 1.2
ak_max_accel = 3.0
ak_angular_error_bias = DegToRad(0)
ak_angular_error_rate = 0.1
