#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
import numpy as np
import wf_param as wf

class Agent:

    def __init__(self):
        pass

    def getState(self,laser_scan):

        laser_scan_disctance = np.average(laser_scan[-15:] + laser_scan[:15])

        # Iterate through the states and check where the laser scan value falls
        for state, (min_distance, max_distance) in wf.STATES.items():
            if min_distance <= laser_scan_disctance <= max_distance:
                return state
        # If the scan value does not fall into any state range, return None
        return None

    def get_action(self, state, q_table):
        action_index = np.argmax(q_table[state])
        action_value = wf.ACTIONS[action_index]
        action = Twist()
        action.angular.z = action_value
        action.linear.y = 0.2
        return action, action_index, action_value
    

    def execute_action(self, action):

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub.publish(action)
        rospy.sleep(0.1)



def main():

    rospy.init_node('wall_follower', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # wait for 5 seconds to allow everything to start up
    rospy.sleep(5)

    agent = Agent()

    while not rospy.is_shutdown():
        laser_scan = rospy.wait_for_message('/scan', LaserScan)
        state = agent.getState(laser_scan.ranges)
        action, action_index, action_value = agent.get_action(state, wf.Q_TABLE) 
        agent.execute_action(action)
        rospy.loginfo(f"State: {state}, Action: {action_index}, distance: {np.average(laser_scan.ranges[-15:] + laser_scan.ranges[:15])}")
        rate.sleep()

if __name__ == '__main__':
    main()