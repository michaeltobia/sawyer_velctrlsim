#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import TransformStamped
from intera_core_msgs.msg import JointCommand
import modern_robotics as mr
import sawyer_MR_description as s_des
import io_util # Not needed unless using catch for user input

#### Intera Joint Control Modes ####
# Must be type int() NOT type Int32() or other std_msgs types
# Intera JointCommand messages are picky
POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)



class VelCtrl:
    def __init__(self):
        # Init node
        rospy.init_node('sim_vel_ctrl')
        # tflistener Init
        self.tf_lis = tf.TransformListener()

        # Trajectory Subscriber
        rospy.Subscriber("/desired_trajectory", TransformStamped, self.ctrl_r)
        # Joint States Subscriber
        rospy.Subscriber("/joint_states", JointState, self.js_store)

        # Control Message Publisher
        self.pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10) ### Gazebo Simulator Only!

        ## Control Gains
        self.Kp = 1*np.eye(6) # Proportional
        self.Ki = 0*np.eye(6) # Intergral

        ## Robot Description
        self.B_list = s_des.Blist
        self.M = s_des.M
        # Set initial joint states to 0, may need to change in IRL use
        # ...but you might not
        self.cur_config = np.zeros(8)

        # Joint Command Message Init
        self.joint_ctrl_msg = JointCommand()
        self.joint_ctrl_msg.names = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        self.joint_ctrl_msg.mode = VELOCITY_MODE
        # Set 0 initial velocity at all joints
        self.joint_ctrl_msg.velocity = \
                    np.ndarray.tolist(np.zeros(len(self.joint_ctrl_msg.names)))

        self.it_count = 0 # Iteration count for debug
        self.int_err = np.zeros(6) # Integrated error 0 -> t




    def ctrl_r(self, X_d):
        # Get current joint positions
        # all 0 if not yet recieved from js_store() callback method below
        cur_theta_list = np.ndarray.tolist(np.delete(self.cur_config, 1))

        # Get desired end endeff transform
        # EndEff frame initially in displacement-quaternion form...
        p_d, Q_d = mr.TFtoMatrix(X_d)
        # ...for ease of use with the Modern Robotics text, the frame is
        # changed to transform matrix form using the tflistener library
        X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)

        # Current EndEff tranform is found using forward kinematics
        X_cur = mr.FKinBody(self.M, self.B_list, cur_theta_list)

        # EndEff transform error X_e is found from [X_e] = log(X^(-1)X_d)
        X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_cur), X_d)))

        # Find integral error
        ## Can induce instability, use with caution!
        self.int_err = (self.int_err + X_e) * (0.5/20)
        # Hard limit each int_err element to 0.5 to prevent runaway
        for i in range(len(self.int_err)):
            err_i = self.int_err[i]
            if abs(err_i) > 0.5:
                self.int_err[i] = np.sign(err_i) * 0.5

        # Calculate body twist command from V_b = K_p . X_e + K_i . int(X_e)
        V_b = np.dot(self.Kp, X_e) + np.dot(self.Ki, self.int_err)
        # Calculate body Jacobian from robot_des and current joint positions
        J_b = mr.JacobianBody(self.B_list, cur_theta_list)

        # Calculate joint velocity command from theta_dot = pinv(J_b)V_b
        J_b_pinv = np.linalg.pinv(J_b)
        theta_dot = np.dot(J_b_pinv, V_b)
        # Limit joint speed to 1 rad/sec
        for i in range(len(theta_dot)):
            if abs(theta_dot[i]) > 1:
                theta_dot[i] = np.sign(theta_dot[i])*1
        print theta_dot
        # Reinsert a 0 joint velocity command for the head_pan joint
        theta_dot = np.insert(theta_dot,1,0)
        # Convert to list because JointCommand messages are PICKY
        self.joint_ctrl_msg.velocity = np.ndarray.tolist(theta_dot)
        # Publish the JointCommand message
        self.pub_joint_ctrl_msg()

    def js_store(self, cur_joint_states):
        # /joint_states subscriber Callback
        # Stores most recent message from /joint_states topic
        # Topic specified in __inti__() method
        self.cur_config = cur_joint_states.position



    def pub_joint_ctrl_msg(self):
        # Add current time stamp to JointCommand message
        self.joint_ctrl_msg.header.stamp = rospy.Time.now() # get header stamp
        # Publish JointCommand message
        self.pub.publish(self.joint_ctrl_msg) # publish joint velocities



if __name__=='__main__':
    try:
        # Init controller
        out = VelCtrl()
        # Publish initial message to get things running smoothly
        out.pub_joint_ctrl_msg()
        rospy.spin()
    except rospy.ROSInterruptException:
        raise e
