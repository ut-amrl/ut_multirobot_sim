NavigationParameters = {
  laser_topic = "/robot1/scan";
  odom_topic = "/robot1/jackal_velocity_controller/odom";
  localization_topic = "/robot1/localization";
  init_topic = "/robot1/initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  laser_loc = {
    x = 0.07;
    y = 0;
  };
  dt = 0.025;
  max_linear_accel = 0.5;
  max_linear_decel = 1.0;
  max_linear_speed = 0.75;
  max_angular_accel = 1.0;
  max_angular_decel = 1.0;
  max_angular_speed = 1.5;
  carrot_dist = 2.5;
  system_latency = 0.24;
  obstacle_margin = 0.1;
  num_options = 41;
  robot_width = 0.44;
  robot_length = 0.5;
  base_link_offset = 0;
  max_free_path_length = 6.0;
  max_clearance = 1.0;
  can_traverse_stairs = false;
  use_map_speed = true;
  target_dist_tolerance = 0.5;
  target_vel_tolerance = 0.1;
};

AckermannSampler = {
  max_curvature = 3.0;
};