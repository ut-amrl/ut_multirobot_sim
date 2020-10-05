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

def make_scenario(dir_name: str, config: dict) -> None:

    # Load Templates
    with open('templates/robot.lua', 'r') as f:
        robot_lua_template = jinja2.Template(f.read())
    
    with open('templates/human.lua', 'r') as f:
        human_lua_template = jinja2.Template(f.read())
    
    with open('templates/sim_config.lua', 'r') as f:
        sim_config_lua_template = jinja2.Template(f.read())

    
    # Create config directory
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)

    # Render and write configs
    with open(dir_name + '/robot.lua', 'w') as f:
        f.write(robot_lua_template.render(config))

    for i in range(config['human_count']):
        if not os.path.exists(dir_name + '/humans'):
            os.mkdir(dir_name + '/humans')
        with open(dir_name + '/humans/human' + str(i) + '.lua', 'w') as f:
            config['waypoints'] = find_random_path()
            f.write(human_lua_template.render(config))

    with open(dir_name + '/sim_config.lua', 'w') as f:
        f.write(sim_config_lua_template.render(config))

@click.command()
@click.option('--count', default=5, help='Number of humans to add')
@click.option('--scenario_name', default='unnamed_scenario', help='name of scenario')
@click.option('--out', default='config', help='Dir to create and put scenario in')
@click.option('--start_x', default=15, help='starting x position for robot')
@click.option('--start_y', default=13, help='starting y position for robot')
@click.option('--start_angle', default=-math.pi / 2, help='starting angle for robot')
def main(count: int, scenario_name: str, out: str, start_x: float, start_y: float, start_angle: float) -> None:
    #show_nav_map()
    config = {
        'start_x': start_x,
        'start_y': start_y,
        'start_angle': start_angle,
        'human_count': count,
        'scenario_name': scenario_name
    }
    make_scenario(out, config)

if __name__ == '__main__':
    main()