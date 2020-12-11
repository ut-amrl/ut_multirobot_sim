#!/usr/bin/env python3

from math import pi
import os

# Format: (x, y, theta)
starting_positions = {
    'left': (3, 2, 0), # left end of hallway
    'right': (38, 5, pi) # right end of the hallway
}

ending_positions = {
    'left': { 'right': (47, 4.5, 0), 'up': (21.5, 9, pi/2) },
    'right': { 'left': (3, 4.5, pi), 'up': (21.5, 9, pi/2)}
}

human_counts = (5, 10, 15)

scenarios_per_hc = 1

for start_pos_name in starting_positions:
    start_x, start_y, start_angle = starting_positions[start_pos_name]
    ending_pos_names = ending_positions[start_pos_name].keys()
    for ending_pos_name in ending_positions[start_pos_name]:
        goal_x, goal_y, goal_angle = ending_positions[start_pos_name][ending_pos_name]
        for human_count in human_counts:
            for i in range(scenarios_per_hc):
                scenario_name = f"sp.{start_pos_name}_ep.{ending_pos_name}_hc.{human_count}_{i}"
                command = "python3 scenario_generator.py " \
                    f"--out ../scenarios/{scenario_name} " \
                    f"--scenario_name {scenario_name} " \
                    f"--count {human_count} " \
                    f"--start_x {start_x} " \
                    f"--start_y {start_y} " \
                    f"--start_angle {start_angle} " \
                    f"--goal_x {goal_x} " \
                    f"--goal_y {goal_y} " \
                    f"--goal_angle {goal_angle} "
                print(scenario_name)
                print(command)
                os.system(command)
