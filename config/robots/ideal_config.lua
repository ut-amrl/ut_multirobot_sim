function Vector3(x, y, z)
  return {x, y, z}  -- Return array-style table for Eigen::Vector3f
end

-- IDEAL ROBOT CONFIGURATION
-- Robot-specific dimensions (no drive model parameters needed)
-- This model applies commands directly without any physical constraints

-- ROBOT GEOMETRY
car_width = 0.7              -- Robot width [m]
car_length = 0.5             -- Robot length [m]
laser_loc = Vector3(0.2, 0.0, 0.0)  -- Laser offset from base_link [m]

-- IDEAL DRIVE MODEL PARAMETERS
-- (No parameters needed - this model has perfect response with no limits)

