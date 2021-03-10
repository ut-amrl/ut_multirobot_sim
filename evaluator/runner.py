#!/usr/bin/env python3

import os
import roslib
import rospy
import signal
import subprocess
import time
import sys

roslib.load_manifest('ut_multirobot_sim')
from amrl_msgs.msg import NavStatusMsg

now = round(time.time())
root = os.getcwd() + '/evals'
policy = "mpdm_synth"
ast_path = ""
#  os.mkdir(root)

def run_scenario(scenario):

    scenario_string = 'scenarios/' + scenario + "/" + scenario + '.launch'
    command = [
        'roslaunch',
        scenario_string,
        'policy:={}'.format(policy),
        'ast_path:={}'.format(ast_path),
        'outfile:={}'.format(root + '/' + scenario + '.json')]
    print(command)
    proc = subprocess.Popen(command, preexec_fn=os.setsid)
    # risk of never killing if goal is never reached
    proc.wait()
    os.system('pkill rosmaster')

def main(scenarios):
    for scenario in scenarios:
        print('running scenario', scenario)
        run_scenario(scenario)

if __name__ == '__main__':
    policy = sys.argv[1]
    ast_path = sys.argv[2]
    scenarios = sys.argv[3:]
    main(scenarios)
