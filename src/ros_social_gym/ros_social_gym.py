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
import json
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

    def __init__(self, reward):
        super(RosSocialEnv, self).__init__()
        seed(1)
        # Halt, GoAlone, Follow, Pass
        self.action_space = spaces.Discrete(4)
        self.action = 0
        self.stepList = []
        self.scores = [0, 0, 0, 0]
        self.startDist = 1.0
        self.lastDist = 1.0
        self.rewardType = reward
        print("Reward Type: " + str(self.rewardType))
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
        self.dataList = []
        self.data = {
                'Iteration': 0,
                'NumHumans': 0,
                'Success': 0,
                'Steps': 0,
                'Data': []
        }

    def __del__(self):
        # Do this on shutdown
        print("Steps: " + str(self.stepCount))
        print("Resets: " + str(self.resetCount))
        string = json.dumps(self.data, indent=2)
        #  fh = open("social_gym.json", 'w', encoding="latin-1")
                #  outputJson.write(string)
        self.launch.shutdown()

    def MakeObservation(self, res):
        obs = MakeNpArray(res.robot_poses)
        obs = np.append(obs, MakeNpArray(res.robot_vels))
        fill_num = ((self.max_humans) - (len(res.human_poses)))
        self.data["NumHumans"] = len(res.human_poses)
        fill_num = 2 * fill_num
        obs = np.append(obs, MakeNpArray(res.human_poses))
        obs = np.append(obs, np.zeros(fill_num))
        obs = np.append(obs, MakeNpArray(res.human_vels))
        obs = np.append(obs, np.zeros(fill_num))
        # Pad with zeroes to reach correct number of variables
        obs = np.append(obs, MakeNpArray([res.goal_pose]))
        obs = np.append(obs, MakeNpArray([res.door_pose]))
        obs = np.append(obs, res.door_state)
        obs = np.append(obs, self.action)
        return obs

    def CalculateReward(self, res, dataMap):
        distance = DistanceFromGoal(res)
        score = (self.lastDist - distance)
        self.lastDist = distance
        force = Force(res)
        blame = Blame(res)
        dataMap['DistScore'] = score
        dataMap['Force'] = force
        dataMap['Blame'] = blame
        bonus = 1.0 if res.done else 0.0
        if (self.rewardType == '0'): # No Social
                return score + bonus
        elif (self.rewardType == '1'): # Nicer
                w1 = 3.0
                w2 = -0.1
                w3 = -0.1
                cost = w1 * score + w2 * Blame(res) + w3 * Force(res)
                return cost + bonus
        elif (self.rewardType == '2'): # Greedier
                w1 = 10.0
                w2 = -0.1
                w3 = -0.1
                cost = w1 * score + w2 * Blame(res) + w3 * Force(res)
                return cost + bonus
        return score + bonus

    def reset(self):
        # Reset the state of the environment to an initial state
        # Call the "reset" of the simulator
        self.resetCount += 1
        self.dataList.append(self.data)
        print(json.dumps(self.dataList, indent=2))
        self.data = {'Iteration': self.resetCount,
                     'NumHumans': 0,
                     'Success': 0,
                     'Steps': 0,
                     'Data': []
                     }
        GenerateScenario()
        response = self.simReset()
        stepResponse = self.simStep(0)
        self.startDist = DistanceFromGoal(stepResponse)
        self.lastDist = self.startDist
        with open('data/SocialGym.json', 'w') as outputJson:
                json.dump(self.dataList, outputJson, indent=2)
        return self.MakeObservation(stepResponse)

    def step(self, action):
        self.stepCount += 1
        self.data["Steps"] = self.stepCount
        tic = time.perf_counter()
        # Execute one time step within the environment
        # Call the associated simulator service (with the input action)
        self.action = action
        response = self.simStep(action)
        toc = time.perf_counter()
        self.stepList.append(toc - tic)
        #  print(f"Average Step Time: {mean(self.stepList)}")
        #  print(f"Step Time: {toc - tic}")
        dataMap = {}
        obs = self.MakeObservation(response)
        obs = [0 if math.isnan(x) else x for x in obs]
        dataMap["Obs"] = obs
        # Calculate Reward in the Python Script
        # Reason, different gyms may have different reward functions,
        # and we don't want to have them need C++.
        # Can be an option later even.
        print("Action: " + str(action))
        reward = self.CalculateReward(response, dataMap)
        print("Reward: " + str(reward))
        self.scores[action] += reward
        print(self.scores)
        done = response.done
        if (done):
                self.data["Success"] = 1
        self.data["Data"].append(dataMap)
        return obs, reward, done, {}

    def render(self):
        print(f'Render')

if __name__ == "__main__":
    env = RosSocialEnv()
    while(True):
        continue
        #  env.step(0)
