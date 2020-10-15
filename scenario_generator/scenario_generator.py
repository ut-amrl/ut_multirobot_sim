#!/usr/bin/env python3

import click
import math
import matplotlib.pyplot as plt
import networkx as nx
import os
import random
import jinja2
from typing import List, Tuple

P = {
    0: (36.67, 19.64),
    1: (28.95, 9.75),
    2: (28.95, 8.75),
    3: (15.18, 8.75),
    4: (15.18, 13.10),
    5: (15.18, 5.35),
    6: (8.70, 8.75)
}

nav_map = nx.DiGraph()
nav_map.add_edges_from((
    (P[0], P[1]),
    (P[1], P[0]),
    (P[1], P[2]),
    (P[2], P[1]),
    (P[2], P[3]),
    (P[3], P[2]),
    (P[3], P[4]),
    (P[3], P[5]),
    (P[3], P[6]),
    (P[4], P[3]),
    (P[5], P[3]),
    (P[6], P[3])
))

top_left = (10, 9.25)
bottom_left = (10, 8.25)
bottom_right = (32, 8.25)
top_right = (32, 9.25)
cycle_map = nx.DiGraph()
cycle_map.add_edges_from((
    (top_left, bottom_left),
    (bottom_left, bottom_right),
    (bottom_right, top_right),
    (top_right, top_left)
))

def show_nav_map() -> None:
    nx.draw(nav_map, with_labels=True, font_weight='bold')
    plt.show()

def find_random_path() -> List[Tuple[float, float]]:
    start_node = random.choice(list(nav_map.nodes()))
    path = [start_node]
    current_node = start_node
    while True:
        neighbors = list(nav_map.neighbors(current_node))
        next_node = random.choice(neighbors)
        print(start_node, next_node)
        if next_node == start_node:
            break
        else:
            current_node = next_node
            path.append(next_node)
    if len(path) < 3:
        return find_random_path()
    else:
        return path

def linear_to_cyclic(linear_displacement):
    linear_displacement = linear_displacement % 46
    if linear_displacement <= 1:
        return (10, 9.25 - linear_displacement)
    elif linear_displacement <= 23:
        return (10 + linear_displacement - 1, 8.25)
    elif linear_displacement <= 24:
        return (32, 8.25 + linear_displacement - 23)
    else:
        return (32 - linear_displacement + 24, 9.25)

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
    
    # Create config directory
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)

    # Make launch file
    with open(dir_name + '/' + config['scenario_name'] + '.launch', 'w') as f:
        f.write(launch_template.render(config))

    # Render and write configs
    with open(dir_name + '/robot.lua', 'w') as f:
        f.write(robot_lua_template.render(config))

    spacing = 46 / config['human_count']
    for i in range(config['human_count']):
        if not os.path.exists(dir_name + '/humans'):
            os.mkdir(dir_name + '/humans')
        linear_displacement = i * spacing
        waypoints = [linear_to_cyclic(linear_displacement+x) for x in range(46)]
        with open(dir_name + '/humans/human' + str(i) + '.lua', 'w') as f:
            config['waypoints'] = waypoints #find_random_path()
            f.write(human_lua_template.render(config))

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
@click.option('--human_speed', default=1.4, help='Human walking speed (m/s)')
def main(count: int, \
        scenario_name: str, \
        out: str, \
        start_x: float, \
        start_y: float, \
        start_angle: float, \
        goal_x: float, \
        goal_y: float, \
        goal_angle: float, \
        human_speed: float) -> None:
    #show_nav_map()
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
        'human_speed': human_speed,
    }
    make_scenario(out, config)

if __name__ == '__main__':
    main()