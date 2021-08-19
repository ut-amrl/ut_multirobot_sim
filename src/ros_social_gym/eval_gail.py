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

from imitation.algorithms import adversarial, bc
from imitation.data import rollout
import pickle

# The algorithms require a vectorized environment to run
env = DummyVecEnv([lambda: RosSocialEnv('1', "config/gym_gen/launch.launch")])
seed(1123)
modelPath = sys.argv[1]
model = None
if (modelPath != ''):
    with open(modelPath, 'rb') as file:
        model = pickle.load(file)

numScenarios = 2000
resetCount = 0
action = [0]
obs, rewards, dones, info = env.step(action)
while resetCount < numScenarios:
    print("Reset Count: " + str(resetCount))
    action = [0]
    if (modelPath != 'ga'):
        action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    resetCount = int(info[0]["resetCount"])
