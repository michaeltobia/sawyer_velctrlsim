#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState  ## Still needed?
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import TransformStamped
from intera_core_msgs.msg import JointCommand
import modern_robotics as mr
import sawyer_MR_description as s_des
import io_util

#### Intera Joint Control Modes ####
POSITION_MODE = Int32(1)
VELOCITY_MODE = Int32(2)
TORQUE_MODE = Int32(3)
TRAJECTORY_MODE = Int32(4)



class VelCtrl:
    def __init__(self):
        ## ROS Init
        rospy.init_node('vel_ctrl')
        self.tf_lis = tf.TransformListener()

        ## Trajectory Subscriber
        self.sub = rospy.Subscriber("/desired_trajectory", TransformStamped, self.ctrl_r)

        #####TESTING PUBLISHERS######
        # self.pub = rospy.Publisher('joint_states', JointState, queue_size=10) ### Rviz traj follower

        #### TESTESTESTESTESTESTESTESTESTESTESTESTEST ######
        self.pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size=10) ### Gazebo Simulator Only!
        ####################################################

        ## Error Publishers
        # self.mean_err_pub = rospy.Publisher('/mean_error', Float64, queue_size=10)
        # self.mean_err_msg = Float64()

        ## Control Gains
        self.Kp = 2*np.eye(6)
        self.Ki = 1*np.eye(6)

        ## Robot Description
        self.B_list = s_des.Blist
        self.M = s_des.M
        self.joint_ctrl_msg = JointCommand()
        self.joint_ctrl_msg.names = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        self.joint_ctrl_msg.mode = VELOCITY_MODE
        self.joint_ctrl_msg.velocity = Float64(np.zeros(len(self.joint_ctrl_msg.names))) # 0 initial velocity


        self.it_count = 0 # Iteration count for debug
        self.int_err = 0 # Integrated error 0 -> t




    def ctrl_r(self, X_d):
        # Get cur' joint positions
        cur_theta_list = np.delete(self.joint_ctrl_msg.velocity, 1)

        # Get EEd transform
        # ##### Need Q_d_dot (for feedforward tho)
        # # Change EEd transform to X_d
        p_d, Q_d = mr.TFtoMatrix(X_d)
        X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)
        # Get cur' X from robot_des and cur' joint positions
        X_cur = mr.FKinBody(self.M, self.B_list, cur_theta_list)
        # Get X_e from [X_e] = log(X^(-1)X_d)
        X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_cur), X_d)))
        # Find integral error (with limit)
        ##### FIX THIS!
        if np.linalg.norm(self.int_err) < 10 and np.linalg.norm(self.int_err) > -10: # int_err limit (-10, 10)
            self.int_err = (self.int_err + X_e)*(0.5/20)

        # Get body twist command V_b = K_p . X_e + K_i . int(X_e)
        V_b = np.dot(self.Kp, X_e) + np.dot(self.Ki, self.int_err)
        # Get body Jacobian from robot_des and cur' joint positions
        J_b = mr.JacobianBody(self.B_list, cur_theta_list)

        # Get joint velocity command from theta_dot = pint(J_b)V_b
        J_b_pinv = np.linalg.pinv(J_b)
        theta_dot = np.dot(J_b_pinv, V_b)

        ##### NOT NEEDED FOR VELOCITY COMMANDS!
        # Position command stuff
        delt_theta = theta_dot*(1.0/20)
        delt_theta = np.insert(delt_theta, 1, 0)
        self.pub_joint_ctrl_msg()



    def pub_joint_ctrl_msg(self):
        ### Get this to produce JointCommand msgs in VelCtrl
        self.joint_ctrl_msg.header.stamp = rospy.Time.now() # get header stamp
        self.pub.publish(self.joint_ctrl_msg) # publish joint velocities
        # self.mean_err_pub.publish(self.mean_err_msg) # publish mean error
        # self.rate.sleep() # sleep for rate



if __name__=='__main__':
    try:
        out = VelCtrl()
        out.pub_joint_ctrl_msg()
        rospy.spin()
        # while not rospy.is_shutdown():
        #     out.pub_joint_ctrl_msg()
    except rospy.ROSInterruptException:
        raise e
