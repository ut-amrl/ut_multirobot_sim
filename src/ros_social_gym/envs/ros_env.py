# Ros Imports
import roslib
roslib.load_manifest('ros_social_env')
import rospy
import roslaunch

# Other Imports
import sys
import gym
import numpy as np
import os
import time


class RosSocialEnv(gym.Env):
    """A ros-based social navigation environment for OpenAI gym"""

    def __init__(self, df):
        super(RosSocialEnv, self).__init__()
        # Initialize as necessary here
        rospy.init_node('RosSocialEnv', anonymous=True)

        # Launch the simulator launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jaholtz/code/ut_multirobot_sim/config/pedsim_example/pedsim_example.launch"])
        self.launch.start()
        rospy.wait_for_service('utmrs_stepper')
        rospy.wait_for_service('utmrs_reset')
        self.simStep = rospy.ServiceProcy('utmrs_stepper', utmrsStepper)
        self.simReset = rospy.ServiceProcy('utmrs_reset', utmrsReset)
        rospy.loginfo("Started Simulator Environment")

    def __del__(self):
        # Do this on shutdown
        self.launch.shutdown()

    def reset(self):
        # Reset the state of the environment to an initial state
        # Call the "reset" of the simulator
        response = self.simReset()

    def step(self, action):
        # Execute one time step within the environment
        # Call the associated simulator service (with the input action)
        response = simStep(action)
        obs = response.obs
        reward = response.reward
        done = response.done

        return obs, reward, done, {}

    def render(self):
        print(f'Render')

