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
env = RosSocialEnv(0)
seed(1123)
model = None
obs = env.reset()

count = 0
max_steps = 2200
max_iters = 500
while count < (max_iters * max_steps):
    obs, rewards, dones, info = env.PipsStep()
    if (dones):
        env.reset()
