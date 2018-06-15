#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState


class RandomJointState:
    def __init__(self):
        rospy.init_node('ref_js_randomizer')
        self.pub = rospy.Publisher('/ref_sawyer/joint_states',JointState, queue_size=10)
        self.ref_js = JointState()
        self.ref_js.name = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        j_ranges = [[-3.05, 3.05],\
                    [0,0],\
                    [-3.81,2.27],\
                    [-3.04,3.04],\
                    [-3.04,3.04],\
                    [-2.98,2.98],\
                    [-2.98,2.98],\
                    [-4.71,4.71]]
        joints = np.ndarray.tolist(np.random.rand(8))
        for i in range(len(joints)):
            joints[i] = joints[i]*np.ptp(j_ranges[i])+j_ranges[i][0]
        self.ref_js.position = joints


    def pub_random_js(self):
        self.ref_js.header.stamp = rospy.Time.now() #add timestamp to joint state msg
        self.pub.publish(self.ref_js)


if __name__=='__main__':
    try:
        out = RandomJointState()
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            out.pub_random_js()
    except rospy.ROSInterruptException:
        raise e
