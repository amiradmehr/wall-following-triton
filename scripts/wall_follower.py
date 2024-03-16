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
import sys

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
            action_index = np.random.choice(len(self.actions))
        else:
            action_index = np.argmax(q_table[state])
            

        return action_index

    def save_q_table(self, q_table):
        # saving the q table in the same directory as the current file
        np.save(self.dir, q_table)

    def load_q_table(self):
        self.q_table = np.load(self.dir, allow_pickle=True)
        # convert the q table to a dictionary
        self.q_table = self.q_table.item()

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
    

    def train(self, robot, gazebo, num_episodes=1000, alpha=0.2, gamma=0.8, epsilon=0.9):

        t_start_train = rospy.Time.now().to_sec()

        for episode in range(num_episodes):

            print(f'time elapsed: {rospy.Time.now().to_sec() - t_start_train:.2f} seconds')


            t_start = rospy.Time.now().to_sec()

            # Reset the environment
            robot.stop()
            # Set the robot to a random position integer between -5 and 3 since the grid is 4x4

            randp_x = np.random.randint(-4, 3) + 0.5 # add 0.5 to make sure the robot is not on the wall
            randp_y = np.random.randint(-4, 3) + 0.5

            # Set the robot to the random position
            gazebo.set_model_state(randp_x, randp_y)
 
            # Get the previous state and model state
            prev_state = self.get_state(robot.ranges)
            prev_model_state = gazebo.get_model_state()
            

            goodjob = 0
            hit_wall = 0

            print(f"Episode {episode + 1} started")

            
            d = 0.998
            eps = epsilon * (d ** episode)
            print(f"epsilon: {eps}")

            for i in range(2000):

                

                # choose a random action
                if np.random.uniform(0, 1) < eps:
                    action_index = np.random.choice(len(self.actions))
                    action = self.actions[action_index]

                else:
                    action_index = self.get_action(self.q_table, prev_state)
                    action = self.actions[action_index]
                # Perform the action
                robot.go_forward()
                robot.steer(angular_speed=action)

                current_state = self.get_state(robot.ranges)
                current_model_state = gazebo.get_model_state()

                # Get the current state and model state

                # print(f'current model state: {current_model_state.pose.position.x, current_model_state.pose.position.y}')
                # print(f'current state: {current_state}')
                # Get the reward
                reward = self.get_reward(prev_state)

                #update the q table
                self.q_table[prev_state][action_index] += alpha * (reward + gamma * np.max(self.q_table[current_state]) - self.q_table[prev_state][action_index])

                # print(f'current model state:\n{current_model_state.pose.position.x, current_model_state.pose.position.y} \n{prev_model_state.pose.position.x, prev_model_state.pose.position.y}')
                
                
                # Check if the robot is hit_wall
                if np.isclose(current_model_state.pose.position.x, prev_model_state.pose.position.x, atol=0.02) and np.isclose(current_model_state.pose.position.y, prev_model_state.pose.position.y, atol=0.02):
                    hit_wall += 1
                    if hit_wall > 4:
                        print("Robot is hit_wall")
                        break
                else:
                    hit_wall = 0


                # Check if the robot is tripped
                r, p, y = euler_from_quaternion([current_model_state.pose.orientation.x, current_model_state.pose.orientation.y, current_model_state.pose.orientation.z, current_model_state.pose.orientation.w])
                if abs(p) > 0.01 or abs(r) > 0.01:
                    rospy.loginfo("Robot is tripped")
                    break

                #check if the robot is doing good; meaning right side is medium and front is not too close
                if current_state[0] == 'medium' and current_state[2] != 'too_close':
                    goodjob += 1

                    if goodjob > 1000:
                        rospy.loginfo("Robot is doing good")
                        break

                if i%100 == 0:
                    rospy.loginfo(f"Episode {episode + 1} step {i} completed")
                
                

                # Update the previous state and model state
                prev_model_state = current_model_state
                prev_state = current_state

                # sleep for a while

                robot.rate.sleep()            
            # Save the q table
            self.save_q_table(self.q_table)
            print(f"Episode {episode + 1} completed")


            t_end = rospy.Time.now().to_sec()

            print(f"Episode {episode + 1} took {t_end - t_start:.2f} seconds")

        t_end_train = rospy.Time.now().to_sec()

        print(f"Training took {t_end_train - t_start_train:.2f} seconds")


    def get_reward(self, state):

        # we want to avoid the wall on the right to be too close or too far
        # also avoid the wall on the left to be too close
        # and also the front wall to be too close
        reward_array = np.zeros(len(self.actions))

        if state[0] == 'too_close' or state[0] == 'too_far' or state[3] == 'close' or state[2] == 'too_close':
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

        self.ranges = {'right': min(msg.ranges[-30:] + msg.ranges[:40]),
                       'front_right': min(msg.ranges[50:70]),
                        'front': min(msg.ranges[85:105]),
                        'left': min(msg.ranges[120:200])}
                        

    def go_forward(self, linear_speed = wf.LINEAR_SPEED_Y):

        self.vel_msg.linear.y = linear_speed
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0

        self.vel_pub.publish(self.vel_msg)

    def steer(self, angular_speed, linear_speed = wf.LINEAR_SPEED_Y):

        #linear speed is a value
        #angular speed is a value in radians and not the index of the action

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

        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(self.model_state_msg.name[0], 'world')
        self.model_state_msg.pose = response.pose
        self.model_state_msg.twist = response.twist

        return response

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

    robot = Robot()

    gaz = Gazebo()

    n = len(sys.argv)

    if n == 2:
        if sys.argv[1] == 'train':
            q.train(robot=robot, gazebo=gaz)
        elif sys.argv[1] == 'run':
            q.load_q_table()
            gaz.set_model_state(3, 0)
            robot.go_forward()
            while not rospy.is_shutdown():
                state = q.get_state(robot.ranges)
                action_index = q.get_action(q.q_table, state)
                action = q.actions[action_index]
                robot.steer(action)
                robot.rate.sleep()
        else:
            print("Invalid argument")


if __name__ == '__main__':

    main()