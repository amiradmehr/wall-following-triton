#! usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
import numpy as np
import wf_param as wf
import time



class QTabel:
    def __init__(self) -> None:

        self.Q_TABLE = {}
        self.states = []


    def initialize_q_table(self, states, actions):
        for right in wf.right_regions:
            for front_right in wf.front_right_regions:
                for front in wf.front_regions:
                    for left in wf.left_regions:
                        self.states.append((right, front_right, front, left))

        for state in self.states:
            self.Q_TABLE[state] = np.zeros(len(wf.ACTIONS))
    
    def update_q_table(self, q_table, state, action, reward, next_state, next_action):
        pass

    def get_action(self, q_table, state):
        # Get the index of the action with the highest value
        action_index = np.argmax(q_table[state])

    def save_q_table(self, q_table):
        pass

    def load_q_table(self, q_table):
        pass

    def get_q_table(self):
        
        pass

    def get_state(right_range, front_right_range, front_range, left_range):
        # Define the range thresholds and corresponding regions
        right_thresholds = [0.2, 0.5, 1.0, float('inf')]
        right_regions = ['too_close', 'medium', 'far', 'too_far']
        front_right_thresholds = [0.5, float('inf')]
        front_right_regions = ['close', 'far']
        front_thresholds = [0.2, 0.5, 1.0, float('inf')]
        front_regions = ['too_close', 'medium', 'far', 'too_far']
        left_thresholds = [0.5, float('inf')]
        left_regions = ['close', 'far']

        # Determine the state for the right side
        right_state = right_regions[max(map(right_range.__ge__, right_thresholds))]

        # Determine the state for the front right side
        front_right_state = front_right_regions[front_right_range >= front_right_thresholds[0]]

        # Determine the state for the front side
        front_state = front_regions[max(map(front_range.__ge__, front_thresholds))]

        # Determine the state for the left side
        left_state = left_regions[left_range >= left_thresholds[0]]

        return (right_state, front_right_state, front_state, left_state)



class Agent:
    def __init__(self) -> None:
        pass




def main():
    pass


if __name__ == '__main__':

    main()