#!/usr/bin/env python

import pandas as pd
import rospy
import roslib
import sys
import subprocess
import time

from threading import Lock

roslib.load_manifest('ut_multirobot_sim')
from amrl_msgs.msg import NavStatusMsg, Pose2Df
from ut_multirobot_sim.msg import HumanStateArrayMsg, Localization2DMsg
from std_msgs.msg import String

robot_position_x = None
robot_position_y = None
robot_velocity_x = None
robot_velocity_y = None
robot_state = None
human_positions = None
human_velocities = None

# now_secs, now_nsecs, robot_pos_x, robot_pos_y, robot_state, human_positions, human_velocities
rows = list()
row_lock = Lock()

def add_row():
    global rows
    now = rospy.Time.now()
    row = (now.secs, now.nsecs, robot_position_x, robot_position_y, robot_velocity_x, robot_velocity_y, robot_state, human_positions, human_velocities)
    row_lock.acquire()
    rows.append(row)
    row_lock.release()

def humans_states_callback(data):
    global human_positions
    global human_velocities
    human_positions = [(state.pose.x, state.pose.y) for state in data.human_states]
    human_velocities = [(state.translational_velocity.x, state.translational_velocity.y) for state in data.human_states]
    add_row()

def localization_callback(data):
    global robot_position_x
    global robot_position_y
    robot_position_x = data.pose.x
    robot_position_y = data.pose.y
    add_row()

def nav_status_callback(data):
    global robot_velocity_x
    global robot_velocity_y
    robot_velocity_x = data.velocity.x
    robot_velocity_y = data.velocity.y
    add_row()

def robot_state_callback(data):
    global robot_state
    robot_state = data.data
    add_row()

def main():
    print('this is the recorder')
    rospy.init_node('recorder')
    out = sys.argv[1]

    def write_json():
        print('making dataframe')
        row_lock.acquire()
        df = pd.DataFrame(rows, columns=['time_secs', 'time_nsecs', 'robot_pos_x', 'robot_pos_y', 'robot_vel_x', 'robot_vel_y', 'robot_state', 'human_positions', 'human_velocities'])
        row_lock.release()
        print('writing dataframe')
        df.to_json(out)

    start_time = rospy.Time.now()

    def wait_for_end(data):
        nav_is_complete = data.nav_complete
        if nav_is_complete or rospy.Time.now() - start_time > rospy.Duration(secs=20):
            time.sleep(0.5)
            subprocess.Popen(['pkill', 'roslaunch'])

    rospy.Subscriber('nav_status', NavStatusMsg, wait_for_end)

    rospy.Subscriber('human_states', HumanStateArrayMsg, humans_states_callback)
    rospy.Subscriber('localization', Localization2DMsg, localization_callback)
    rospy.Subscriber('nav_status', NavStatusMsg, nav_status_callback)
    rospy.Subscriber('robot_state', String, robot_state_callback)

    rospy.on_shutdown(write_json)
    rospy.spin()

if __name__ == '__main__':
    main()