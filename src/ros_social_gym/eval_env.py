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
env = DummyVecEnv([lambda: RosSocialEnv(0)])
seed(1123)
modelPath = sys.argv[1]
model = None
if (modelPath != 'ga'):
    model = PPO.load(modelPath)
obs = env.reset()
#obs = env.reset()
#obs = env.reset()

numScenarios = 100
resetCount = 0
while resetCount < numScenarios:
    action = [0]
    if (modelPath != 'ga'):
        action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    resetCount = int(info[0]["resetCount"])
