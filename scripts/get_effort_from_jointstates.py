#! /usr/bin/env python2

import sys
import copy
import rospy
#import input
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

rospy.init_node('get_effort_from_jointstates', anonymous=True)
effort_command_publisher = rospy.Publisher('panda1/effort_jointgroup_controller/command', Float64MultiArray, queue_size=1)

time_last_sent = rospy.Time.now()
interval = rospy.Duration(0.1)

command = Float64MultiArray()
command.layout.dim.append(MultiArrayDimension(size=7))
command.data = [0.0] * 7


def joint_state_callback(data):
    global time_last_sent, command
    current_time = rospy.Time.now()
    if current_time - time_last_sent >= interval:
        command.data = data.effort
        print(command)
        effort_command_publisher.publish(command)
        time_last_sent = current_time


joint_state_subscriber = rospy.Subscriber('panda1/joint_states', JointState, joint_state_callback)


if __name__ == '__main__':
    rospy.spin()
