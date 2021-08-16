import gym
import json
from random import seed
import datetime as dt
#  from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
#  import pandas as pdj
from ros_social_gym import RosSocialEnv
from make_scenarios import GenerateScenario
import sys

# The algorithms require a vectorized environment to run
reward = sys.argv[1]
env = DummyVecEnv([lambda: RosSocialEnv(reward, "config/gdc_gym_gen/launch.launch")])
seed(1)
#  GenerateScenario()
env.reset()
model = DQN("MlpPolicy", env, verbose=0)
#  model.save("data/ppo_base")
#  model = PPO.load("base_ppo", env)
count = 0
upper = 1000
while(count < upper):
  model.learn(total_timesteps=2000, log_interval=4)
  # Save the agent
  model.save("data/dqn_" + str(count))
  count += 1
