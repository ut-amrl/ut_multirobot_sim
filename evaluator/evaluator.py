#!/usr/bin/env python3

import numpy as np
import time
import rospy
import roslib

roslib.load_manifest('ut_multirobot_sim')
from amrl_msgs.msg import NavStatusMsg, Pose2Df
from ut_multirobot_sim.msg import HumanStateArrayMsg, Localization2DMsg
from std_msgs.msg import String
from pprint import pprint

class Tracker:
    def __init__(self, name):
        self.name = name
    
    def report(self):
        raise NotImplementedError

class TimeToGoalTracker(Tracker):
    def __init__(self, name):
        super().__init__(name)
        rospy.Subscriber('nav_status', NavStatusMsg, self.nav_status_callback)
        self.start_time = rospy.Time.now()
        self.end_time = None

    def report(self):
        if self.end_time is None:
            return float('inf')
        delta = (self.end_time - self.start_time)
        return delta.secs + (delta.nsecs / 10e9)

    def nav_status_callback(self, data):
        if self.end_time is not None:
            return
        nav_is_complete = data.nav_complete
        if nav_is_complete:
            self.end_time = rospy.Time.now()

class SpeedStatsTracker(Tracker):
    def __init__(self, name):
        super().__init__(name)
        rospy.Subscriber('nav_status', NavStatusMsg, self.nav_status_callback)
        self.speeds = []

    def report(self):
        return {
            'mean': np.mean(self.speeds)
        }
    
    def nav_status_callback(self, data):
        x = data.velocity.x
        y = data.velocity.y
        speed = np.linalg.norm([x, y])
        self.speeds.append(speed)

class CollisionCountTracker(Tracker):
    def __init__(self, name):
        super().__init__(name)
        rospy.Subscriber('localization', Localization2DMsg, self.position_update)
        rospy.Subscriber('human_states', HumanStateArrayMsg, self.human_update)
        self.collision_count = 0
        self.robot_pos = None
        self.currently_colliding_with = []

    def position_update(self, data):
        self.robot_pos = [data.pose.x, data.pose.y]
    
    def human_update(self, data):
        if self.robot_pos is None:
            return
        human_states = data.human_states
        if self.currently_colliding_with == []:
            self.currently_colliding_with = [False] * len(human_states)
        for human_id, state in enumerate(human_states):
            human_pos = np.array([state.pose.x, state.pose.y])
            if np.linalg.norm(human_pos - self.robot_pos) < 0.5: # not reliable
                if not self.currently_colliding_with[human_id]:
                    self.collision_count += 1
                    self.currently_colliding_with[human_id] = True
            else:
                if self.currently_colliding_with[human_id]:
                    self.currently_colliding_with[human_id] = False

    def report(self):
        return self.collision_count

class MinDistanceToHumanTracker(Tracker):
    def __init__(self, name):
        super().__init__(name)
        rospy.Subscriber('localization', Localization2DMsg, self.position_update)
        rospy.Subscriber('human_states', HumanStateArrayMsg, self.human_update)
        self.min_dist = float('inf')
        self.robot_pos = None

    def position_update(self, data):
        self.robot_pos = [data.pose.x, data.pose.y]

    def human_update(self, data):
        if self.robot_pos is None:
            return
        human_states = data.human_states
        for state in human_states:
            human_pos = np.array([state.pose.x, state.pose.y])
            distance = np.linalg.norm(human_pos - self.robot_pos)
            self.min_dist = min(self.min_dist, distance)

    def report(self):
        return self.min_dist

class TimeInStateTracker(Tracker):
    def __init__(self, name):
        super().__init__(name)
        rospy.Subscriber('robot_state', String, self.state_update)
        self.current_state = None
        self.times = {}
        self.start_times = {}

    def state_update(self, data):
        state_name = data.data
        if state_name != self.current_state: # switched to new state
            if self.current_state is not None:
                self.times[self.current_state] = self.times[self.current_state] + (rospy.Time.now() - self.start_times[self.current_state])
            self.current_state = state_name
            self.start_times[state_name] = rospy.Time.now()
            if state_name not in self.times.keys():
                self.times[state_name] = rospy.Duration(0)
        
        print(rospy.Time.now() - self.start_times[state_name])
        self.times[state_name] = self.times[state_name] + (rospy.Time.now() - self.start_times[state_name])
    
    def report(self):
        ret = {}
        if self.current_state is not None:
            self.times[self.current_state] = self.times[self.current_state] + (rospy.Time.now() - self.start_times[self.current_state])
        for state in self.times.keys():
            ret[state] = self.times[state].secs + self.times[state].nsecs / 10e9
        return ret

class SpeedMatchingTracker(Tracker):
    def __init__(self, name, closeness_threshold):
        super().__init__(name)
        self.closeness_threshold = closeness_threshold
        self.speed_differences = []
        rospy.Subscriber('human_states', HumanStateArrayMsg, self.human_update)
        rospy.Subscriber('localization', Localization2DMsg, self.pos_update)
        rospy.Subscriber('nav_status', NavStatusMsg, self.vel_update)
        self.robot_pos = None
        self.robot_vel = None

    def pos_update(self, data):
        self.robot_pos = [data.pose.x, data.pose.y]

    def vel_update(self, data):
        x = data.velocity.x
        y = data.velocity.y
        self.robot_vel = np.linalg.norm([x, y])

    def human_update(self, data):
        if self.robot_pos is None or self.robot_vel is None:
            return
        human_states = data.human_states
        for state in human_states:
            human_pos = np.array([state.pose.x, state.pose.y])
            human_vel = np.linalg.norm([state.translational_velocity.x, state.translational_velocity.y])
            distance = np.linalg.norm(human_pos - self.robot_pos)
            if distance < self.closeness_threshold:
                self.speed_differences.append(self.robot_vel - human_vel)

    def report(self):
        return {
            'mean': np.mean(self.speed_differences),
            'stdev': np.std(self.speed_differences)
        }


def report_evaluation(trackers):
    evaluation = {}
    for tracker in trackers:
        evaluation[tracker.name] = tracker.report()
    pprint(evaluation)

def main():
    rospy.init_node('evaluator')
    ttg_track = TimeToGoalTracker('time_to_goal')
    ss_track = SpeedStatsTracker('speed_stats')
    col_track = CollisionCountTracker('collision_count')
    mdth_track = MinDistanceToHumanTracker('min_distance_to_human')
    tis_track = TimeInStateTracker('time_in_states')
    sm_track = SpeedMatchingTracker('difference_in_speed_with_near_humans', 2)
    rospy.on_shutdown(lambda: report_evaluation([ttg_track, ss_track, col_track, mdth_track, tis_track, sm_track]))

    print('hello')

    rospy.spin()


if __name__ == '__main__':
    main()