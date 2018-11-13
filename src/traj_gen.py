#!/usr/bin/eng python

import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Vector3
from math import sin, cos, pi

class TrajectoryGenerator:
    def __init__(self):
        rospy.init_node('ref_trajectory')
        self.pub_motion = rospy.Publisher('/desired_trajectory',
                            TransformStamped, queue_size=10)
        self.pub_force = rospy.Publisher('/desired_force', Vector3,
                            queue_size=10)
        self.desired_trajectory_frame = TransformStamped()
        self.desired_trajectory_frame.child_frame_id = "base"
        self.desired_force = Vector3()

    def trajectory(self, t):
        x_d = 0.5*cos(2*pi*t/5)
        y_d = 0.5*sin(2*pi*t/5)
        return x_d, y_d

    def publish_trajectory(self):
        t = rospy.Time.now()
        self.desired_trajectory_frame.header.stamp = t
        x_d, y_d = self.trajectory(t)
        self.desired_trajectory_frame.transform.translation = [x_d, y_d, 0]
        self.desired_trajectory_frame.transform.rotation = [0,0,0,0]
        self.desired_force = [0,0,1]
        self.pub_motion.publish(self.desired_trajectory_frame)
        self.pub_force.publish(self.desired_force)
        rate.sleep()

if __name__=='__main__':
    try:
        out = TrajectoryGenerator()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            out.publish_trajectory()
    except rospy.ROSInterruptException:
        raise e
