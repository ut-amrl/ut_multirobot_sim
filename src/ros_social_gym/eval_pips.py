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
launch = sys.argv[1]
env = RosSocialEnv(0, launch)
seed(1123)
model = None
obs = env.reset()

numScenarios = 11
resetCount = 0
while resetCount < numScenarios:
    obs, rewards, dones, info = env.PipsStep()
    resetCount = int(info["resetCount"])
    if (dones):
        env.reset()
