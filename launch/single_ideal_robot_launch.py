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

    return LaunchDescription([
        # Simulator node
        launch_ros.actions.Node(
            package='ut_multirobot_sim',
            executable='simulator',
            name='simulator',
            cwd=ut_multirobot_sim_dir,
            arguments=[
                '--',
                '--config', 'config/environment/sim_config.lua',
                '--maps_dir', amrl_maps_dir
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
                '--',
                '-robot_config', 'config/navigation.lua'
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
                '--',
                '--config_file=config/webviz_config.lua'
            ],
            output='screen'
        )
    ])
