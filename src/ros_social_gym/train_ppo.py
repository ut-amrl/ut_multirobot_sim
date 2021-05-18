import gym
import json
from random import seed
import datetime as dt
#  from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
#  import pandas as pdj
from ros_social_gym import RosSocialEnv
from make_scenarios import GenerateScenario
import sys

# The algorithms require a vectorized environment to run
reward = sys.argv[1]
env = DummyVecEnv([lambda: RosSocialEnv(reward)])
seed(1)
#  GenerateScenario()
env.reset()
#  model = PPO("MlpPolicy", env, verbose=0)
count = 0
upper = 1
model = PPO.load("first_model", env)
while(count < upper):
        model.learn(total_timesteps=20000)
        # Save the agent
        model.save("data/ppo_" + str(count))
        count += 1
env.reset()
