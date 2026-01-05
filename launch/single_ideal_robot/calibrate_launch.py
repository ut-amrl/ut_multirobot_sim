from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Find package directories
    ut_multirobot_sim_dir = Path(get_package_share_directory('ut_multirobot_sim'))
    amrl_maps_dir = get_package_share_directory('amrl_maps')
    graph_navigation_dir = Path(get_package_share_directory('graph_navigation'))
    webviz_dir = Path(get_package_share_directory('webviz'))
    launch_dir = ut_multirobot_sim_dir / 'launch' / 'single_ideal_robot'
    sim_config = launch_dir / 'sim_config.lua'
    nav_config = launch_dir / 'navigation.lua'
    webviz_config = launch_dir / 'webviz_config.lua'

    # use_sim_time=True for simulation (nodes read /clock instead of wall clock)
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # Simulator node (does NOT use sim time - it publishes /clock)
        Node(
            package='ut_multirobot_sim',
            executable='simulator',
            name='simulator',
            cwd=str(ut_multirobot_sim_dir),
            arguments=[
                '--config', str(sim_config),
                '--maps_dir', str(amrl_maps_dir),
                '--'  # stop gflags parsing before ROS args
            ],
            output='screen'
        ),

        # Navigation node (remapped to avoid interfering with calibration)
        Node(
            package='graph_navigation',
            executable='navigation',
            name='navigation',
            cwd=str(graph_navigation_dir),
            parameters=[use_sim_time],
            arguments=[
                '-robot_config', str(nav_config),
                '--'  # stop gflags parsing before ROS args
            ],
            remappings=[
                ('/robot0/cmd_vel', '/robot0/cmd_vel_dump')  # Remap nav commands to different topic
            ],
            output='screen'
        ),

        # WebViz node
        Node(
            package='webviz',
            executable='websocket',
            name='websocket',
            cwd=str(webviz_dir),
            parameters=[use_sim_time],
            arguments=[
                f'--config_file={webviz_config}',
                '--'  # stop gflags parsing before ROS args
            ],
            output='screen'
        ),

        # Actuation latency calibration script (uses sim time)
        Node(
            package='ut_multirobot_sim',
            executable='calibrate_actuation_latency.py',
            name='calibrate_actuation_latency',
            parameters=[use_sim_time],
            output='screen'
        )
    ])
