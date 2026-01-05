websocket = {
  port = 10272;                    -- WebSocket server port
  update_rate_hz = 40.0;           -- Visualization update rate (fps)
  message_timeout_sec = 2.0;       -- Maximum age before messages are dropped
  exit_check_interval_ms = 100;    -- Timer interval for exit signal checking
  current_map_publish_rate_hz = 1.0; -- Rate to publish current map name
};

ros_topics = {
  -- Input topics (subscribers)
  laser_scan = "/robot0/scan";
  visualization = "/visualization";
  visualization_local = "/visualization_local";
  localization = "/robot0/localization";
  nav_status = "/navigation_goal_status";

  -- Output topics (publishers)
  initial_pose_std = "/initialpose";           -- Standard ROS nav stack
  nav_goal_std = "/move_base_simple/goal";     -- Standard ROS nav stack
  initial_pose_amrl = "/robot0/initialpose";   -- AMRL format
  nav_goal_amrl = "/set_nav_target";           -- AMRL format
  reset_nav_goals = "/reset_nav_goals";        -- Reset command
  current_map = "/current_map";                -- Current active map name
};

ros_node = {
  name = "websocket";              -- ROS node name
  queue_sizes = {
    laser_scan = 20;                -- Laser scan subscriber queue
    visualization = 20;            -- Visualization subscriber queue  
    localization = 20;             -- Localization subscriber queue
    nav_status = 20;               -- Navigation status subscriber queue
    publishers = 20;               -- All publisher queues
  };
};

-- Frame Configuration
frames = {
  robot_frame = "robot0/base_link";       -- Robot-relative frame
  world_frame = "map";             -- World/global frame
};

-- Data Processing Configuration
data_processing = {
  protocol_nonce = 42;             -- Binary protocol identifier
  text_buffer_size = 32;           -- Max characters for text annotations
  map_name_buffer_size = 32;       -- Max characters for map names
};

-- Logging Configuration
logging = {
  verbosity = 0;                   -- Default verbosity level (0=minimal, 1=info, 2=debug)
};
  
-- Performance Tuning
performance = {
  enable_message_aging = true;     -- Drop old messages based on timestamp
  enable_rate_limiting = true;     -- Limit update rate to configured fps
  thread_sleep_usec = 100000;      -- Microseconds to sleep before thread cleanup
}; 
