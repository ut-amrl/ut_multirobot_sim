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

# The algorithms require a vectorized environment to run
env = DummyVecEnv([lambda: RosSocialEnv()])
seed(1)
#  GenerateScenario()
env.reset()
model = PPO("MlpPolicy", env, verbose=0)
model.learn(total_timesteps=20000000)
# Save the agent
model.save("test_ppo")
#  obs = env.reset()
#  model.learn(total_timesteps=5)
