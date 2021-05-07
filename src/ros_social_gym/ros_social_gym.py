#!/usr/bin/env python

# Ros Imports
import roslib
NODE_NAME = 'ros_social_gym'
roslib.load_manifest(NODE_NAME)
from ut_multirobot_sim.srv import utmrsStepper
from ut_multirobot_sim.srv import utmrsReset
from make_scenarios import GenerateScenario
import rospy
import roslaunch

# Other Imports
import sys
import gym
from gym import spaces
import numpy as np
import os
import time
import math
from statistics import mean
from random import seed


def MakeNpArray(poseList):
        coordList = []
        for pose in poseList:
            coordList.append(pose.x)
            coordList.append(pose.y)
        return np.array(coordList)

def ClosestHuman(robot_pose, poses):
    best_dist = 9999
    best_pose = robot_pose
    for pose in poses:
        pose_array = np.array([pose.x, pose.y])
        distance = np.linalg.norm(robot_pose - pose_array)**2
        if (distance < best_dist):
            best_dist = distance
            best_pose = pose_array
    return best_pose, best_dist

# TODO(jaholtz, special handling for follow)
def Force(response):
    #  robot_state = response.
    robot_pose = response.robot_poses[0]
    robot_pose = np.array([robot_pose.x, robot_pose.y])
    human_poses = response.human_poses
    if (response.robot_state == 2):
            del human_poses[response.follow_target]
    # Find the closest human to the robot
    closest_distance = ClosestHuman(robot_pose, response.human_poses)[1]
    force = np.exp(-closest_distance**2 / 5)
    return force

def sigmoid(x):
    return 1 - math.erf(x)

def closest_point_on_line_segment_to_point(end1, end2, point):
    l2 = np.linalg.norm(end1 - end2)**2
    if np.abs(l2) < 1e-6:
        return end1
    t = max(0, min(1, np.dot(point - end1, end2 - end1) / l2))
    projection = end1 + t * (end2 - end1)
    return projection

def Blame(response):
    # Find the closest human
    robot_pose = response.robot_poses[0]
    robot_pose = np.array([robot_pose.x, robot_pose.y])
    human, closest_distance = ClosestHuman(robot_pose, response.human_poses)

    # forward predicted robot position
    robot_vel = response.robot_vels[0]
    robot_vel = [robot_vel.x, robot_vel.y]
    end2 = robot_pose + (np.array(robot_vel) * 0.5)

    # closest point to human
    closest_point = closest_point_on_line_segment_to_point(robot_pose, end2, human)

    # blame
    blame = sigmoid(np.linalg.norm(closest_point - human))
    return blame

def DistanceFromGoal(response):
    robot_pose = MakeNpArray(response.robot_poses)
    goal_pose = MakeNpArray([response.goal_pose])
    return np.linalg.norm(robot_pose[0] - goal_pose[0])


class RosSocialEnv(gym.Env):
    """A ros-based social navigation environment for OpenAI gym"""

    def __init__(self):
        super(RosSocialEnv, self).__init__()
        seed(1)
        # Halt, GoAlone, Follow, Pass
        self.action_space = spaces.Discrete(4)
        self.action = 0
        self.stepList = []
        # TODO(Make this parameterized by the number of humans and robots)
        # goal_x, goal_y, robot_1x, robot_1y, ... ,
        # robot_nx, robot_ny, robot_1vx, robot_1vy, ...,
        # human_1x, human_1y, ..., human_1vx, human_1vy ...
        # next_door_x, next_door_y, next_door_state
        self.num_robots = 1
        self.max_humans = 40
        self.length = 6 + (self.num_robots*4) + (self.max_humans * 4)
        self.feat_length = (self.num_robots*4) + (self.max_humans * 4)
        self.observation_space = spaces.Box(low=-9999,
                                            high=9999,
                                            shape=(self.length,))
        # Initialize as necessary here
        rospy.init_node('RosSocialEnv', anonymous=True)

        # Launch the simulator launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #  self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jaholtz/code/ut_multirobot_sim/config/pedsim_example/pedsim_example.launch"])
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jaholtz/code/ut_multirobot_sim/config/gdc_gym_gen/launch.launch"])
        self.launch.start()
        rospy.wait_for_service('utmrsStepper')
        rospy.wait_for_service('utmrsReset')
        self.simStep = rospy.ServiceProxy('utmrsStepper', utmrsStepper)
        self.simReset = rospy.ServiceProxy('utmrsReset', utmrsReset)
        self.resetCount = 0
        self.stepCount = 0

    def __del__(self):
        # Do this on shutdown
        print("Steps: " + str(self.stepCount))
        print("Resets: " + str(self.resetCount))
        self.launch.shutdown()

    def MakeObservation(self, res):
        obs = MakeNpArray(res.robot_poses)
        obs = np.append(obs, MakeNpArray(res.robot_vels))
        obs = np.append(obs, MakeNpArray(res.human_poses))
        obs = np.append(obs, MakeNpArray(res.human_vels))
        # Pad with zeroes to reach correct number of variables
        print(self.feat_length)
        print(len(obs))
        fill_num = self.feat_length - len(obs)
        obs = np.append(obs, np.zeros(fill_num))
        obs = np.append(obs, MakeNpArray([res.goal_pose]))
        obs = np.append(obs, MakeNpArray([res.door_pose]))
        obs = np.append(obs, res.door_state)
        obs = np.append(obs, self.action)
        return obs

    def CalculateReward(self, res):
        w1 = 1.0
        w2 = 1.0
        w3 = 1.0
        cost = w1 * Blame(res) + w2 * Force(res) +  w3 * DistanceFromGoal(res)
        #  print('Reward' + str(-DistanceFromGoal(res)))
        return -DistanceFromGoal(res)

    def reset(self):
        # Reset the state of the environment to an initial state
        # Call the "reset" of the simulator
        self.resetCount += 1
        GenerateScenario()
        response = self.simReset()
        return self.step(0)[0]

    def step(self, action):
        self.stepCount += 1
        tic = time.perf_counter()
        # Execute one time step within the environment
        # Call the associated simulator service (with the input action)
        self.action = action
        response = self.simStep(action)
        toc = time.perf_counter()
        self.stepList.append(toc - tic)
        #  print(f"Average Step Time: {mean(self.stepList)}")
        #  print(f"Step Time: {toc - tic}")
        obs = self.MakeObservation(response)
        obs = [0 if math.isnan(x) else x for x in obs]
        # Calculate Reward in the Python Script
        # Reason, different gyms may have different reward functions,
        # and we don't want to have them need C++.
        # Can be an option later even.
        reward = self.CalculateReward(response)
        done = response.done

        return obs, reward, done, {}

    def render(self):
        print(f'Render')

if __name__ == "__main__":
    env = RosSocialEnv()
    while(True):
        continue
        #  env.step(0)
