from math import radians
import numpy as np

LINEAR_SPEED_Y = 0.3


# Define the state space
states = []

# 1. Right side divided into 4 regions
right_regions = ['too_close', 'close', 'medium', 'far', 'too_far']
right_thresholds = [0.1, 0.2, 0.5, 1.0, np.inf]

# 2. Front right divided into 2 regions
front_right_regions = ['close', 'far']
front_right_thresholds = [1.5, np.inf]

# 3. Front side divided into 4 regions
front_regions = ['too_close', 'close', 'medium', 'far']
front_thresholds = [0.1, 0.4, 1.0, 2.0, np.inf]

# 4. Left divided into 2 regions
left_regions = ['close', 'far']
left_regions = [0.5, np.inf]

'''
actions are defined as the angle to turn the robot
'''
# Right, Forward, Left
ACTIONS = {'right_fast': radians(-30), 'right_slow': radians(-15), 
           'forward': 0, 
           'left_fast': radians(30), 'left_slow': radians(15)}
print(len(ACTIONS))



'''
states are defined as the distance to the wall on the right side of the robot
1 range of angles are defined for the lidar sensor
-15 15 at each range the distance to the wall is categorized 
as close, optimal and far
close is defined as a distance less than 0.5 meters
medium is defined as a distance between 0.5 and 0.51 meters
far is defined as a distance greater than 0.51 and inf meters
'''


# Define the range of angles for the lidar sensor
angle_ranges = [(-30, 15)]

# Define the STATES variable
STATES = {'close': (0,0.5), 'optimal': (0.5,0.51), 'far': (0.51, float('inf'))}

# Q-learning parameters

Q_TABLE = {
    'close': [-5, -10, -10],
    'optimal': [-10, 10, -10],
    'far': [-10, -10, -5]
}


