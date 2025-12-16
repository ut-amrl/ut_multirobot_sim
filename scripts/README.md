# Scenario generator (human crowd)

This folder only contains the helper to emit human-crowd scenarios as Lua configs the simulator can load.

## Files
- `example.yml`: editable template describing a crowd scenario (map name, robot start, human start/goal pairs).
- `generate_config.py`: converts a YAML file into Lua configs.
- `human_config_template.txt`, `init_config_template.txt`: string templates used by the generator.

## Usage
```bash
python scripts/generate_config.py --config-file scripts/example.yml
```
Outputs:
- `config/scenarios/human_crowd/<prefix>/init_config.lua`
- `config/scenarios/human_crowd/<prefix>/human/human_config_*.lua`

## Using the generated configs
- Point the simulator at the generated init file with `--init_config config/scenarios/human_crowd/<prefix>/init_config.lua`
  **or** set `init_config_file` inside `config/environment/sim_config.lua` to that path.
- Keep `--env_config` and `--robot_config` as usual; the generated files only cover init poses and human definitions.
