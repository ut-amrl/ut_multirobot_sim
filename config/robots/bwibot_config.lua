-- MODEL PARAMETERS
invert_linear_vel_cmds = false
invert_angular_vel_cmds = false
linear_pos_accel_limit = 3.0
linear_neg_accel_limit = 3.0
angular_pos_accel_limit = 3.0
angular_neg_accel_limit = 3.0
max_angular = 3.0
max_linear_vel = 3.0
drive_callback_topic = "/cmd_vel"
diff_drive_odom_topic = "/odom"
linear_odom_scale = 1.0
angular_odom_scale = 1.0


-- SIMULATOR PARAMETERS

-- tf
publish_map_to_odom = true
publish_foot_to_base = true

-- Kinematic
rear_axle_offset = 0.0
min_turn_radius = 0.0

laser_loc = Vector3(0.15, 0, 0.155)
car_width = 0.34
car_length = 0.34
car_height = 1.5;

