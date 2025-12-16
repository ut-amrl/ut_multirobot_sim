import argparse
from pathlib import Path
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"


def parse_args():
    parser = argparse.ArgumentParser(description="Generate a human crowd scenario under config/scenarios/human_crowd/")
    parser.add_argument(
        "--config-file",
        default="scripts/example.yml",
        type=str,
        help="Path to a YAML scenario description (default: scripts/example.yml)",
    )
    return parser.parse_args()


def load_yaml(path: Path):
    with path.open("rb") as f:
        return yaml.load(f, yaml.FullLoader)


def main():
    args = parse_args()

    config_path = Path(args.config_file)
    if not config_path.is_absolute():
        config_path = ROOT / config_path

    data = load_yaml(config_path)

    prefix = data["prefix"]
    scenario_dir = ROOT / "config" / "scenarios" / "human_crowd" / prefix
    human_dir = scenario_dir / "human"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    human_dir.mkdir(parents=True, exist_ok=True)

    init_template = (SCRIPTS_DIR / "init_config_template.txt").read_text()
    human_template = (SCRIPTS_DIR / "human_config_template.txt").read_text()

    human_start_goal_positions = data["human_start_goal_positions"]
    map_name = data["map_name"]
    start_x = data["start_x"]
    start_y = data["start_y"]
    start_angle = data["start_angle"]

    human_config_paths = []
    for i, (start, goal) in enumerate(human_start_goal_positions):
        config_file = human_dir / f"human_config_{i}.lua"
        config_file.write_text(
            human_template.format(start[0], start[1], goal[0], goal[1])
        )
        human_config_paths.append(f"\"{config_file.as_posix()}\"")

    init_config_path = scenario_dir / "init_config.lua"
    init_config_path.write_text(
        init_template.format(
            map_name,
            start_x,
            start_y,
            start_angle,
            ",\n".join(human_config_paths),
        )
    )

    print(f"Generated {init_config_path} and {len(human_config_paths)} human configs in {human_dir}")
    print("Use this init_config via --init_config or set init_config_file in config/environment/sim_config.lua.")


if __name__ == "__main__":
    main()
