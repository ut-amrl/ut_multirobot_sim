#!/bin/bash

CONFIG_FILE="double_jackal_config.lua"
# CONFIG_FILE="sim_config.lua"

./bin/simulator \
 --localize="true" \
 --sim_config="config/$CONFIG_FILE"