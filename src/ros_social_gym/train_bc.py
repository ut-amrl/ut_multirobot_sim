"""Loads CartPole-v1 demonstrations and trains BC, GAIL, and AIRL models on that data.
"""

import pathlib
import pickle
import tempfile

import stable_baselines3 as sb3

from imitation.algorithms import adversarial, bc
from imitation.data import rollout
from imitation.util import logger, util
from torch.utils.data import Dataset, DataLoader
from social_dataset import SocialDataset
from ros_social_gym import RosSocialEnv
from stable_baselines3.common.vec_env import DummyVecEnv

# create Environtment
venv = DummyVecEnv([lambda: RosSocialEnv(1, "config/gdc_gym_gen/launch.launch")])

# Train BC on expert data.
# BC also accepts as `expert_data` any PyTorch-style DataLoader that iterates over
# dictionaries containing observations and actions.
transitions = SocialDataset('big_bc_demos.json')
# Generate 100 models to evaluate, yay...
minSamples = len(transitions // 100)
maxSamples = len(transitions)
for i in range(minSamples, maxSamples, maxSamples // 100):
    transitions = SocialDataset('big_bc_demos.json', i)
    logger.configure("/home/jaholtz/code/imitation/temp/BC/")
    bc_trainer = bc.BC(venv.observation_space, venv.action_space, expert_data=transitions)
    bc_trainer.train(n_epochs=10)
    bc_trainer.save_policy('bc_policy_' + str(i))
