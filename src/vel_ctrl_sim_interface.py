#!/usr/bin/env python

import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand

class SimInterface:
    def __init__(self):
        # Init node
        rospy.init_node('vel_ctrl_sim_interface')
        # Init joint_states puslisher for sim
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # Init joint_command subscriber for JointCommand control messages
        rospy.Subscriber('/robot/limb/right/joint_command', JointCommand, self.commandToState)
        self.joint_positions = JointState() # Init JointState message
        self.joint_positions.name = ['right_j0', 'head_pan', 'right_j1',\
                                     'right_j2', 'right_j3', 'right_j4',\
                                     'right_j5', 'right_j6']
        # Set all initial joint positions at 0 (might not be needed)
        self.joint_positions.position = np.ndarray.tolist(np.zeros(8))

    def commandToState(self, vel_command):
        vel_command = np.asarray(vel_command.velocity) # convert to np.array()
        pos_command = vel_command*(1.0/20) # velocity/rate = position
        pos_command[1] = 0 # ensure 0 head rotation
        self.joint_positions.position += pos_command # increment position
        self.joint_positions.header.stamp = rospy.Time.now() # stamp joint pos'
        self.pub.publish(self.joint_positions) # publish joint positions

if __name__=='__main__':
    try:
        out = SimInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        raise e
