import json
import sys
import os
import numpy as np

run = sys.argv[1]

files = os.listdir(run)

goal_times = []

for file in files:
    with open(run + '/' + file) as f:
        x = json.loads(f.read())
        time_to_goal = x['time_to_goal']
        goal_times.append(time_to_goal)

print('mean\t', np.mean(goal_times))
print('stdev\t', np.std(goal_times))
print('n\t', len(goal_times))