#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand

class SimInterface:
    def __init__(self):
        rospy.init_node('vel_ctrl_sim_interface')
        self.pub = rospy.Publisher('/joint_states')
        self.sub = rospy.Subscriber('robot/limb/right/joint_command',\
                                    JointCommand, queue_size = 10,\
                                    self.commandToState)
        self.joint_positions.name = ['right_j0', 'head_pan', 'right_j1',\
                                     'right_j2', 'right_j3', 'right_j4',\
                                     'right_j5', 'right_j6']
        self.joint_positions.position = Float64(np.zeros(8)) # init pos @ all 0

    def commandToState(self, vel_command):
        vel_command = vel_command.velocity
        pos_command = vel_command*(1.0/20) # velocity/rate = position
        pos_command[1] = 0 # ensure 0 head rotation
        self.joint_positions.position += pos_command()
        self.joint_positions.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_positions)

if __name__=='__main__':
    try:
        out = SimInterface()
        out.commandToState()
        rospy.spin()
    except rospy.ROSInterruptException:
        raise e
