import os, sys
import argparse
import numpy as np
import yaml

dir = os.path.dirname(__file__)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-file', default="example.yml", type=str)
    return parser.parse_args()

def main():

    args = parse_args()

    with open(dir + "/" + args.config_file, 'rb') as f:
        data = yaml.load(f, yaml.FullLoader)

    config = {}
    for key, value in data.items():
        if isinstance(value, list):
            config[key] = []
            for v in value:
                print(v)
                config[key].append(value)
        else:
            config[key] = value

    prefix = config["prefix"]
    folder_path = f"config/human_crowd_scenario_configs/{prefix}"
    os.makedirs(folder_path, exist_ok=True)

    with  open(dir + "/init_config_template.txt", "r") as f:
        init_config_str = f.read()
    
    with  open(dir + "/human_config_template.txt", "r") as f:
        human_config_str = f.read()

    human_start_goal_positions = config["human_start_goal_positions"][0]
    map_name = config["map_name"]
    start_x = config["start_x"]
    start_y = config["start_y"]
    start_angle = config["start_angle"]
    num_human = len(human_start_goal_positions)

    os.makedirs(f"{folder_path}/human", exist_ok=True)
    # Genereate human data

    human_config_file_str = ""
    for i in range(num_human):
        config_file_name = f"{folder_path}/human/human_config_{i}.lua"
        with open(config_file_name, "w") as f:
            f.write(human_config_str.format(human_start_goal_positions[i][0][0], human_start_goal_positions[i][0][1],
                                      human_start_goal_positions[i][1][0], human_start_goal_positions[i][1][1]))
        human_config_file_str += f"\"{config_file_name}\" "
        if i != num_human - 1:
            human_config_file_str += ",\n\r"

    # generate init_config
    with open(f"{folder_path}/init_config.lua", "w") as f:
        f.write(init_config_str.format(map_name, start_x, start_y, start_angle, human_config_file_str))

            
if __name__ == "__main__":
    main()
