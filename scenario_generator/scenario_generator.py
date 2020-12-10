#!/usr/bin/env python3

import click
import math
import os
import jinja2
import random


def make_scenario(dir_name: str, config: dict) -> None:

    # Load Templates
    with open('templates/robot.lua', 'r') as f:
        robot_lua_template = jinja2.Template(f.read())

    with open('templates/human.lua', 'r') as f:
        human_lua_template = jinja2.Template(f.read())

    with open('templates/sim_config.lua', 'r') as f:
        sim_config_lua_template = jinja2.Template(f.read())

    with open('templates/launch.launch', 'r') as f:
        launch_template = jinja2.Template(f.read())

    with open('templates/pedsim_launch.launch', 'r') as f:
        pedsim_launch_template = jinja2.Template(f.read())

    with open('templates/scene.xml', 'r') as f:
        scene_xml_template = jinja2.Template(f.read())

    # Create config directory
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)

    # Make launch file
    with open(dir_name + '/' + config['scenario_name'] + '.launch', 'w') as f:
        f.write(launch_template.render(config))

    # Make pedsim launch file
    with open(dir_name + '/pedsim_launch.launch', 'w') as f:
        f.write(pedsim_launch_template.render(config))

    # Render and write configs
    with open(dir_name + '/robot.lua', 'w') as f:
        f.write(robot_lua_template.render(config))

    human_positions = list()

    for i in range(config['human_count']):
        if not os.path.exists(dir_name + '/humans'):
            os.mkdir(dir_name + '/humans')
        position_x = random.uniform(-0.5, 45)
        position_y = 2 if random.choice((True, False)) else 5
        human_positions.append((position_x, position_y))
        waypoints = [(position_x, position_y)]
        with open(dir_name + '/humans/human' + str(i) + '.lua', 'w') as f:
            config['waypoints'] = waypoints  # find_random_path()
            f.write(human_lua_template.render(config))

    config['human_positions'] = human_positions
    with open(dir_name + '/scene.xml', 'w') as f:
        f.write(scene_xml_template.render(config))

    with open(dir_name + '/sim_config.lua', 'w') as f:
        f.write(sim_config_lua_template.render(config))


@click.command()
@click.option('--count', default=5, help='Number of humans to add')
@click.option('--scenario_name', help='name of scenario')
@click.option('--out', default='config', help='Folder to create and put scenario in')
@click.option('--start_x', default=15.0, help='starting x position for robot')
@click.option('--start_y', default=13.0, help='starting y position for robot')
@click.option('--start_angle', default=-math.pi / 2, help='starting angle for robot')
@click.option('--goal_x', default=15.0, help='ending x position for robot')
@click.option('--goal_y', default=13.0, help='ending y position for robot')
@click.option('--goal_angle', default=-math.pi / 2, help='ending angle for robot')
def main(count: int,
         scenario_name: str,
         out: str,
         start_x: float,
         start_y: float,
         start_angle: float,
         goal_x: float,
         goal_y: float,
         goal_angle: float) -> None:
    if scenario_name is None:
        print('you fool! you forgot the scenario name!')
        exit(1)

    config = {
        'start_x': start_x,
        'start_y': start_y,
        'start_angle': start_angle,
        'goal_x': goal_x,
        'goal_y': goal_y,
        'goal_angle': goal_angle,
        'human_count': count,
        'scenario_name': scenario_name,
    }
    make_scenario(out, config)


if __name__ == '__main__':
    main()
