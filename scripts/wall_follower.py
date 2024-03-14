#! /usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
import pandas as pd
import numpy as np
import wf_param as wf
import os

class QTabel:
    def __init__(self) -> None:

        self.q_table = {}
        self.states = []
        self.actions = list(wf.ACTIONS.values())
        self.q_table_df = None


    def initialize_q_table(self, states, actions):
        for right in wf.right_regions:
            for front_right in wf.front_right_regions:
                for front in wf.front_regions:
                    for left in wf.left_regions:
                        self.states.append((right, front_right, front, left))

        for state in self.states:
            self.q_table[state] = np.zeros(len(wf.ACTIONS))
        
        self.q_table_df = pd.DataFrame(self.q_table.values(), index=self.q_table.keys(), columns=wf.ACTIONS.keys())
    
    def update_q_table(self, q_table, state, action, reward, next_state, next_action):
        pass

    def get_action(self, q_table, state):
        # Get the index of the action with the highest value
        action_index = np.argmax(q_table[state])

        return action_index

    def save_q_table(self, q_table):
        np.save('q_table.npy', q_table)

    def load_q_table(self, q_table):
        self.q_table = np.load('q_table.npy', allow_pickle=True)

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
    



class Robot:
    def __init__(self) -> None:

        #initialize the node
        rospy.init_node('Triton', anonymous=True)

        self.ranges= None
        self.lidar_topic = '/scan'
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)


        self.vel_topic = '/cmd_vel'
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.vel_msg = Twist()

        self.model_name = 'triton'

        self.rate = rospy.Rate(10)

        while self.ranges is None:
            self.rate.sleep()

    def lidar_callback(self, msg: LaserScan):

        self.ranges = {'right': min(msg.ranges[-60:] + msg.ranges[:60]),
                       'front_right': min(msg.ranges[60:80]),
                        'front': min(msg.ranges[85:105]),
                        'left': min(msg.ranges[120:180])}


    def go_forward(self, linear_speed = wf.LINEAR_SPEED_Y):

        self.vel_msg.linear.y = linear_speed
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0

        self.vel_pub.publish(self.vel_msg)

    def steer(self, angular_speed, linear_speed = wf.LINEAR_SPEED_Y):
        self.vel_msg.linear.y = linear_speed
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = angular_speed

        self.vel_pub.publish(self.vel_msg)

    def stop(self):
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.x = 0

        self.vel_pub.publish(self.vel_msg)



class Gazebo:
    def __init__(self) -> None:
        self.model_state_msg = ModelStates()
        self.model_state_msg.name = ['triton']
        self.get_model_state()

    def get_model_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state(self.model_state_msg.name[0], 'world')
            self.model_state_msg.pose = response.pose
            self.model_state_msg.twist = response.twist
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def set_model_state(self, x , y):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.model_state_msg.name[0]
            model_state.pose.position.x = x
            model_state.pose.position.y = y

            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = set_model_state(model_state)
            
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


def main():

    



if __name__ == '__main__':

    main()