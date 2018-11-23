#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Wrench, Vector3, Quaternion
from math import sin, cos, pi

class TrajectoryGenerator:
    def __init__(self):
        rospy.init_node('ref_trajectory')
        self.pub_motion = rospy.Publisher('/desired_trajectory',
                            TransformStamped, queue_size=10)
        self.pub_wrench = rospy.Publisher('/desired_wrench', Wrench,
                            queue_size=10)
        self.desired_trajectory_frame = TransformStamped()
        self.desired_trajectory_frame.child_frame_id = "base"
        self.desired_wrench = Wrench()

    def trajectory(self, t):
        x_d = 0.5*cos(2*pi*t/5)
        y_d = 0.5*sin(2*pi*t/5)
        z_d = 0.5
        return x_d, y_d, z_d

    def publish_trajectory(self):
        self.desired_trajectory_frame.header.stamp = rospy.Time.now()
        x_d, y_d, z_d = self.trajectory(rospy.get_time())
        self.desired_trajectory_frame.transform.translation = Vector3(x_d,y_d,z_d)
        self.desired_trajectory_frame.transform.rotation = Quaternion(0,0,0,0)
        self.desired_wrench.force = Vector3(0,0,0)
        self.desired_wrench.torque = Vector3(0,0,0)
        self.pub_motion.publish(self.desired_trajectory_frame)
        self.pub_wrench.publish(self.desired_wrench)
        rate.sleep()

if __name__=='__main__':
    try:
        out = TrajectoryGenerator()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            out.publish_trajectory()
    except rospy.ROSInterruptException:
        raise e
