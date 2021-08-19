import gym
import sys
import json
from random import seed
import datetime as dt
#  from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
#  import pandas as pdj
from ros_social_gym import RosSocialEnv
from make_scenarios import GenerateScenario

# The algorithms require a vectorized environment to run
env = RosSocialEnv('1', 1, 'config/gdc_gym_gen/greedy_launch.launch')
seed(1)
model = None
#  obs = env.reset()

numScenarios = 2000
resetCount = 0
while resetCount < numScenarios:
    print("Reset Count: " + str(resetCount))
    obs, rewards, dones, info = env.PipsStep()
    resetCount = int(info["resetCount"])
    if (dones):
        env.reset()
