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

        self.dir = os.path.join(os.path.dirname(__file__), 'q_table.npy')

    def initialize_q_table(self):
        for right in wf.right_regions:
            for front_right in wf.front_right_regions:
                for front in wf.front_regions:
                    for left in wf.left_regions:
                        self.states.append((right, front_right, front, left))

        for state in self.states:
            self.q_table[state] = np.zeros(len(wf.ACTIONS))
        
        # self.q_table_df = pd.DataFrame(self.q_table.values(), index=self.q_table.keys(), columns=wf.ACTIONS.keys())
    
    def update_q_table(self, q_table, state, action, reward, next_state, next_action):
        pass

    def get_action(self, q_table, state):
        # Get the index of the action with the highest value
        # if every value is the same then choose a random action
        if np.all(q_table[state] == q_table[state][0]):
            print(f"q_table[state]: {q_table[state]}")
            action_index = np.random.choice(len(self.actions))
        else:
            action_index = np.argmax(q_table[state])
            

        return action_index

    def save_q_table(self, q_table):
        # saving the q table in the same directory as the current file
        np.save(self.dir, q_table)

    def load_q_table(self, q_table):
        self.q_table = np.load(self.dir, allow_pickle=True)

    def get_state(self, lidar_ranges):

        right_range = lidar_ranges['right']
        front_right_range = lidar_ranges['front_right']
        front_range = lidar_ranges['front']
        left_range = lidar_ranges['left']

        # Determine the state for the right side
        right_state_index = np.argmax(right_range < wf.right_thresholds)
        right_state = wf.right_regions[right_state_index]

        # Determine the state for the front right side
        front_right_state_index = np.argmax(front_right_range < wf.front_right_thresholds)
        front_right_state = wf.front_right_regions[front_right_state_index]

        # Determine the state for the front side
        front_state_index = np.argmax(front_range < wf.front_thresholds)
        front_state = wf.front_regions[front_state_index]

        # Determine the state for the left side
        left_state_index = np.argmax(left_range < wf.left_thresholds)
        left_state = wf.left_regions[left_state_index]

        return (right_state, front_right_state, front_state, left_state)
    

    def train(self, robot, q_table, gazebo, num_episodes=1000, alpha=0.2, gamma=0.9, epsilon=0.1):

        for episode in range(num_episodes):
            # Reset the environment
            robot.stop()
            # Set the robot to a random position integer between -5 and 3 since the grid is 4x4

            randp_x = np.random.randint(-5, 3) + 0.5 # add 0.5 to make sure the robot is not on the wall
            randp_y = np.random.randint(-5, 3) + 0.5

            # Set the robot to the random position
            gazebo.set_model_state(randp_x, randp_y)
            
            # Get the current state
            state = self.get_state(robot.ranges)

            # Get the action
            action = self.get_action(q_table, state)

            # Initialize the total reward
            total_reward = 0

            while True:
                # Perform the action
                robot.steer(self.actions[action])

                # Get the next state
                right_range, front_right_range, front_range, left_range = robot.ranges.values()
                next_state = self.get_state(right_range, front_right_range, front_range, left_range)

                # Get the reward
                reward = self.get_reward(next_state)

                # Get the next action
                next_action = self.get_action(q_table, next_state)

                # Update the Q-table
                q_table[state][action] += alpha * (reward + gamma * q_table[next_state][next_action] - q_table[state][action])

                # Update the state and action
                state = next_state
                action = next_action

                # Update the total reward
                total_reward += reward

                # Check if the episode is done
                if self.is_done(state):
                    break

            # Print the total reward
            print(f"Episode {episode + 1}: Total Reward: {total_reward}")

            # Save the Q-table
            self.save_q_table(q_table)

    def get_reward(self, state):

        # we want to avoid the wall on the right to be too close or too far
        # also avoid the wall on the left to be too close
        # and also the front wall to be too close
        if state[0] == 'too_close' or state[0] == 'too_far' or state[3] == 'too_close' or state[2] == 'too_close':
            return -1
        else:
            return 0

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
                        'left': min(msg.ranges[120:180])}\
                        

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

    def run_from_q_table(self, q_table):
        pass



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

    q = QTabel()
    q.initialize_q_table()

    print(q.q_table[('too_close', 'close', 'too_close', 'close')][2])

    # q.q_table[('too_close', 'close', 'too_close', 'close')][2] = 3
    print(q.q_table[('too_close', 'close', 'too_close', 'close')][2])

    a = q.get_action(q.q_table, ('too_close', 'close', 'too_close', 'close'))
    print(f"action: {a}")

    sample_lidar_ranges = {'right': 0.1, 'front_right': 1.4, 'front': 0.7, 'left': 0.5}

    print(q.get_state(sample_lidar_ranges))



if __name__ == '__main__':

    main()