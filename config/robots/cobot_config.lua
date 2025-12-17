function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

function DegToRad(d)
  return math.pi * d / 180
end

-- COBOT ROBOT CONFIGURATION
-- Robot-specific dimensions and drive model parameters

-- ROBOT GEOMETRY
car_width = 0.4
car_length = 0.4
car_height = 0.15
rear_axle_offset = 0.0
laser_loc = Vector3(0.0, 0.0, 0.15)

-- OMNIDIRECTIONAL DRIVE MODEL PARAMETERS
co_base_radius = 0.2
co_w0 = DegToRad(45.0)
co_w1 = DegToRad(135.0)
co_w2 = DegToRad(-135.0)
co_w3 = DegToRad(-45.0)
co_max_speed = 1.2
co_max_accel = 3.0
co_max_angle_vel = math.pi
co_max_angle_accel = math.pi
