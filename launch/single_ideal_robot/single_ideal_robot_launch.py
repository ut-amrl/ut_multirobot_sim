import launch
import launch_ros.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Find package directories
    ut_multirobot_sim_dir = get_package_share_directory('ut_multirobot_sim')
    amrl_maps_dir = get_package_share_directory('amrl_maps')
    graph_navigation_dir = get_package_share_directory('graph_navigation')
    webviz_dir = get_package_share_directory('webviz')
    launch_dir = os.path.join(ut_multirobot_sim_dir, 'launch', 'single_ideal_robot')
    sim_config = os.path.join(launch_dir, 'sim_config.lua')
    nav_config = os.path.join(launch_dir, 'navigation.lua')
    webviz_config = os.path.join(launch_dir, 'webviz_config.lua')

    return LaunchDescription([
        # Simulator node
        launch_ros.actions.Node(
            package='ut_multirobot_sim',
            executable='simulator',
            name='simulator',
            cwd=ut_multirobot_sim_dir,
            arguments=[
                '--config', sim_config,
                '--maps_dir', amrl_maps_dir,
                '--'  # stop gflags parsing before ROS args
            ],
            output='screen'
        ),

        # Navigation node
        launch_ros.actions.Node(
            package='graph_navigation',
            executable='navigation',
            name='navigation',
            cwd=graph_navigation_dir,
            arguments=[
                '-robot_config', nav_config,
                '-clearance_weight', '-0.5',
                '-freepath_weight', '-1.0',
                '-subopt_tolerance', '1.5',
                '--'  # stop gflags parsing before ROS args
            ],
            output='screen'
        ),

        # WebViz node
        launch_ros.actions.Node(
            package='webviz',
            executable='websocket',
            name='websocket',
            cwd=webviz_dir,
            arguments=[
                f'--config_file={webviz_config}',
                '--'  # stop gflags parsing before ROS args
            ],
            output='screen'
        )
    ])
