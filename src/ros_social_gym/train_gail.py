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
from social_gail_dataset import SocialDataset
from ros_social_gym import RosSocialEnv
from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.policies import serialize
import torch as th
import os
from random import seed

def save(trainer, save_path):
    """Save discriminator and generator."""
    # We implement this here and not in Trainer since we do not want to actually
    # serialize the whole Trainer (including e.g. expert demonstrations).
    os.makedirs(save_path, exist_ok=True)
    th.save(trainer.discrim, os.path.join(save_path, "discrim.pt"))
    # TODO(gleave): unify this with the saving logic in data_collect?
    # (Needs #43 to be merged before attempting.)
    serialize.save_stable_model(
        os.path.join(save_path, "gen_policy"),
        trainer.gen_algo,
        trainer.venv_norm_obs,
    )

# create Environtment
venv = DummyVecEnv([lambda: RosSocialEnv(1, 20, "config/gym_gen/launch.launch")])
seed(1)

# Train BC on expert data.
# BC also accepts as `expert_data` any PyTorch-style DataLoader that iterates over
# dictionaries containing observations and actions.
transitions = SocialDataset('big_bc_demos.json')
log_dir = "/root/gail_training/"
logger.configure(log_dir)
#  bc_trainer = bc.BC(venv.observation_space, venv.action_space, expert_data=transitions)
#  bc_trainer.train(n_epochs=100)
#  bc_trainer.save_policy('bc_policy_100')

# Train GAIL on expert data.
# GAIL, and AIRL also accept as `expert_data` any Pytorch-style DataLoader that
# iterates over dictionaries containing observations, actions, and next_observations.
#  logger.configure("/home/jaholtz/code/imitation/temp/BC/")
gail_trainer = adversarial.GAIL(
    venv,
    expert_data=transitions,
    expert_batch_size=100,
    gen_algo=sb3.PPO("MlpPolicy", venv, verbose=1, n_steps=100),
)

#  checkpoint_interval = len(transitions) // 100
checkpoint_interval = 100
print(len(transitions))
print(checkpoint_interval)

def callback(round_num):
    if checkpoint_interval > 0 and round_num % checkpoint_interval == 0:
        save(gail_trainer, os.path.join(log_dir, "checkpoints", f"{round_num:05d}"))

gail_trainer.train(len(transitions), callback)

#  # Train AIRL on expert data.
#  logger.configure(tempdir_path / "AIRL/")
#  airl_trainer = adversarial.AIRL(
    #  venv,
    #  expert_data=transitions,
    #  expert_batch_size=32,
    #  gen_algo=sb3.PPO("MlpPolicy", venv, verbose=1, n_steps=1024),
#  )
#  airl_trainer.train(total_timesteps=2048)
