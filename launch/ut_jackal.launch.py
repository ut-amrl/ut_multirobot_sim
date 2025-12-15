from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ut_multirobot_sim')
    
    env_config = LaunchConfiguration('env_config', default=os.path.join(pkg_share, 'config', 'sim_config.lua'))
    robot_config = LaunchConfiguration('robot_config', default=os.path.join(pkg_share, 'config', 'ut_jackal_config.lua'))
    init_config = LaunchConfiguration('init_config', default=os.path.join(pkg_share, 'config', 'default_init_config.lua'))
    
    return LaunchDescription([
        DeclareLaunchArgument('env_config', default_value=env_config),
        DeclareLaunchArgument('robot_config', default_value=robot_config),
        DeclareLaunchArgument('init_config', default_value=init_config),
        
        Node(
            package='ut_multirobot_sim',
            executable='simulator',
            name='simulator',
            output='screen',
            arguments=[
                '--env_config', env_config,
                '--robot_config', robot_config,
                '--init_config', init_config
            ],
            remappings=[
                ('/robot0/jackal_velocity_controller/odom', 'jackal_velocity_controller/odom'),
                ('/robot0/scan', 'velodyne_2dscan'),
                ('/robot0/navigation/cmd_vel', 'cmd_vel'),
            ]
        )
    ])

