from math import radians
import numpy as np
import pandas as pd


LINEAR_SPEED_Y = 0.3



# Define the state space
states = []

# 1. Right side divided into 4 regions
right_regions = np.array(['too_close', 'close', 'medium', 'far', 'too_far'])
right_thresholds = np.array([0.3, 0.4, 0.5, 1.0, np.inf])

# 2. Front right divided into 2 regions
front_right_regions = np.array(['close', 'far'])
front_right_thresholds = np.array([1.5, np.inf])

# 3. Front side divided into 4 regions
front_regions = np.array(['too_close', 'close', 'medium', 'far'])
front_thresholds = np.array([0.3, 0.5, 1.3, np.inf])

# 4. Left divided into 2 regions
left_regions = np.array(['close', 'far'])
left_thresholds = np.array([0.5, np.inf])

'''
actions are defined as the angle to turn the robot
'''
# Right, Forward, Left
ACTIONS = {'right_fast': radians(-100), 'right_slow': radians(-30), 
           'forward': 0, 
           'left_fast': radians(100), 'left_slow': radians(30)}


# Combine all regions to create the state space
for right in right_regions:
    for front_right in front_right_regions:
        for front in front_regions:
            for left in left_regions:
                states.append((right, front_right, front, left))


# Create the Q-table
q_table = {}
for state in states:
    q_table[state] = np.zeros(len(ACTIONS))

# print(q_table[states[0]])


# turn it into pandas dataframe
q_table_df = pd.DataFrame(q_table.values(), index=q_table.keys(), columns=ACTIONS.keys())

# print(q_table_df.head())

# query the first state

# print(q_table_df.loc[('too_close', 'close', 'too_close', 'close')]['right_fast'])