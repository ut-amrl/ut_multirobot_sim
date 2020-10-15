#!/usr/bin/env python3

from math import pi
import os

# Format: (x, y, theta)
starting_positions = {
    'rd': (29, 11, -pi/2), # rightmost doorway for "main" hall
    'ld': (15, 12, -pi/2), # leftmost doorway
    'fh': (7, 9, 0)       # part of the hallway past where humans are walking
}

ending_positions = {
    'rd': { 'ld': (15, 12, pi/2), 'fh': (7, 9, pi) },
    'ld': { 'rd': (29, 11, pi/2), 'fh': (7, 9, pi) },
    'fh': { 'rd': (29, 11, pi/2), 'ld': (15, 12, pi/2) }
}

human_speeds = (1, 1.5, 2) # m/s

human_counts = (1, 5, 10, 15, 20, 25)

for start_pos_name in starting_positions:
    start_x, start_y, start_angle = starting_positions[start_pos_name]
    ending_pos_names = ending_positions[start_pos_name].keys()
    for ending_pos_name in ending_positions[start_pos_name]:
        goal_x, goal_y, goal_angle = ending_positions[start_pos_name][ending_pos_name]
        for human_speed in human_speeds:
            for human_count in human_counts:
                scenario_name = f"sp.{start_pos_name}_ep.{ending_pos_name}_hs.{human_speed}_hc.{human_count}"
                command = "python scenario_generator.py " \
                    f"--out ../scenarios/{scenario_name} " \
                    f"--scenario_name {scenario_name} " \
                    f"--count {human_count} " \
                    f"--start_x {start_x} " \
                    f"--start_y {start_y} " \
                    f"--start_angle {start_angle} " \
                    f"--goal_x {goal_x} " \
                    f"--goal_y {goal_y} " \
                    f"--goal_angle {goal_angle} " \
                    f"--human_speed {human_speed}"
                print(scenario_name)
                print(command)
                os.system(command)