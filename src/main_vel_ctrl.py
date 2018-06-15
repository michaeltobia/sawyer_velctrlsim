#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState
import modern_robotics as mr
import sawyer_MR_description as s_des

class MainVelCtrl:
    def __init__(self):
        rospy.init_node('main_vel_ctrl')
        self.tf_lis = tf.TransformListener()
        self.pub = rospy.Publisher('/main_sawyer/joint_states', JointState, queue_size=10)
        self.Kp = 1
        self.Ki = 1
        self.B_list = s_des.Blist
        self.M = s_des.M
        self.main_js = JointState()
        self.main_js.name = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        self.main_js.position = np.ndarray.tolist(np.zeros(8)) #init main_js


    def ctrl_r(self):
        try:
            (p_d, Q_d) = self.tf_lis.lookupTransform('/ref_tf/base', '/ref_tf/right_hand', rospy.Time(0))
        except tf.LookupException:
            continue
        self.cur_theta_list = np.delete(self.main_js.position, 1)
        self.X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)
        self.X = mr.FKinBody(self.M, self.B_list, self.cur_theta_list)
        self.X_e = mr.MatrixLog6(np.dot(mr.TransInv(X), X_d))
        self.V_b = self.Kp*self.X_e
        ## THETA DOT NEXT


    def pub_main_js(self):
        self.main_js.header.stamp = rospy.Time.now()
        self.pub.publish(self.main_js)
        rate.sleep()

if __name__=='__main__':
    try:
        out = MainVelCtrl()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            out.pub_main_js()
    except rospy.ROSInterruptException:
        raise e
