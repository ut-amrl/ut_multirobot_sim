# Hallway simulation

## Generate custom scenario

1. Follow the `example.yml` to fill in prefix, initial position,
   positions for human etc.

1. Generate config files
Please use Python `>=3.6`

``` shell
python scripts/generate_config.py --config-file CONFIG_FILENAME
```
This will generate config files under `config` folder.

1. Go to `config/sim_config.lua` , and replace the init files
