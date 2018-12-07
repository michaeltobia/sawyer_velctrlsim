#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
import modern_robotics as mr
import sawyer_MR_description as s_des
import io_util


# class SimpleSubEx:
#     def __init__(self):
#         self.traj_sub = rospy.Subscriber('desired_trajectory', TransformStamped, self.testmethod)
#
#     def testmethod(self, X_d):
#         print X_d




class VelCtrl:
    def __init__(self):
        ## ROS Init
        rospy.init_node('vel_ctrl')
        self.tf_lis = tf.TransformListener()
        self.sub = rospy.Subscriber("/desired_trajectory", TransformStamped, self.ctrl_r)


        #####TESTING PUBLISHERS######
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10) ### Rviz traj follower

        #### TESTESTESTESTESTESTESTESTESTESTESTESTEST ######
        # self.pub = rospy.Publisher('robot/limb/right/joint_command', JointState, queue_size=10) ### Gazebo Simulator Only!
        ####################################################

        # self.rate = rospy.Rate(20) # 20Hz
        ## Trajectory Subscriber
        ## Error Publishers
        # self.mean_err_pub = rospy.Publisher('/mean_error', Float64, queue_size=10)
        # self.mean_err_msg = Float64()
        ## Control Gains
        self.Kp = 2*np.eye(6)
        self.Ki = 1*np.eye(6)
        ## Robot Description
        self.B_list = s_des.Blist
        self.M = s_des.M
        self.main_js = JointState()
        self.main_js.name = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']

        ## Joint velocity message
        # self.main_js.velocity = np.ndarray.tolist(np.zeros(8)) #init main_js
        ## Joint position
        self.main_js.position = np.ndarray.tolist(np.zeros(8))
        self.pub_main_js()

        self.it_count = 0 # Iteration count for debug
        self.int_err = 0 # Integrated error 0 -> t




    def ctrl_r(self, X_d):
        # (p_cur, Q_cur) = self.tf_lis.lookupTransform('/main_tf/base', '/main_tf/right_hand', rospy.Time(0))
        # X_cur = self.tf_lis.fromTranslationRotation(p_cur, Q_cur)
        # # Get cur' joint positions
        cur_theta_list = np.delete(self.main_js.position, 1)
        # Get EEd transform

        ####### Now pulling X_d from traj_gen
        # ##### Change to trajectory pull!
        # (self.p_d, self.Q_d) = self.tf_lis.lookupTransform('/ref_tf/base', '/ref_tf/right_hand', rospy.Time(0))
        # ##### Need Q_d_dot (for feedforward tho)
        # # Change EEd transform to X_d
        p_d, Q_d = mr.TFtoMatrix(X_d)
        X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)
        # Get cur' X from robot_des and cur' joint positions
        X_cur = mr.FKinBody(self.M, self.B_list, cur_theta_list)
        # Get X_e from [X_e] = log(X^(-1)X_d)
        self.X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_cur), X_d)))
        # Find integral error (with limit)

        ##### FIX THIS!
        if np.linalg.norm(self.int_err) < 10 and np.linalg.norm(self.int_err) > -10: # int_err limit (-10, 10)
            self.int_err = (self.int_err + self.X_e)*(0.5/20) # Add int_err straight to X_e?

        # Get body twist command V_b = K_p . X_e + K_i . int(X_e)
        self.V_b = np.dot(self.Kp, self.X_e) + np.dot(self.Ki, self.int_err)
        # Get body Jacobian from robot_des and cur' joint positions
        self.J_b = mr.JacobianBody(self.B_list, cur_theta_list)
        # Hybrid Control Prototype (surprise, doesnt work)
        T_selector = np.matrix([[0,0,0],\
                               [0,0,0],\
                               [1,0,0],\
                               [0,1,0],\
                               [0,0,1],\
                               [0,0,0]])
        Y_selector = np.matrix([[1,0,0],\
                               [0,1,0],\
                               [0,0,0],\
                               [0,0,0],\
                               [0,0,0],\
                               [0,0,1]])
        # Get joint velocity command from theta_dot = pint(J_b)V_b
        J_b_pinv = np.linalg.pinv(self.J_b)
        theta_dot = np.dot(J_b_pinv, self.V_b)
        coeff_null = (np.eye(7) - np.dot(self.J_b.T, J_b_pinv.T))
        theta_dot += np.dot(coeff_null, theta_dot)
        # theta_dot = np.dot(np.linalg.pinv(self.J_b), np.dot(self.V_b, Y_selector))

        # self.main_js.velocity = self.theta_dot # set new joint velocities
        # self.main_js.header.stamp = rospy.Time.now() # get header stamp
        # self.pub.publish(self.main_js)

        ##### NOT NEEDED FOR VELOCITY COMMANDS!
        # Position command stuff
        delt_theta = theta_dot*(1.0/20)
        delt_theta = np.insert(delt_theta, 1, 0)
        self.main_js.position += delt_theta
        self.pub_main_js()


        # ###### TESTESTESTESTESTESTESTESTESTESTESTESTEST ##############
        # #TESTING VELOCITY COMMANDS
        # # Position command stuff
        # delt_theta = theta_dot
        # delt_theta = np.insert(delt_theta, 1, 0)
        # self.main_js.position += delt_theta
        # self.pub_main_js()
        # ###########################################################

        # self.mean_err_msg.data = np.mean(self.X_e) ## FIX THIS
        # print self.mean_err_msg.data ## DEBUG PRINT
        # print np.linalg.norm(self.int_err) ## FIX THIS

        # print(self.theta_dot)

        # THETA DOT NEXT


    def pub_main_js(self):
        self.main_js.header.stamp = rospy.Time.now() # get header stamp
        self.pub.publish(self.main_js) # publish joint velocities
        # self.mean_err_pub.publish(self.mean_err_msg) # publish mean error
        # self.rate.sleep() # sleep for rate



if __name__=='__main__':
    try:
        out = VelCtrl()
        out.pub_main_js()
        rospy.spin()
        # while not rospy.is_shutdown():
        #     out.pub_main_js()
    except rospy.ROSInterruptException:
        raise e
