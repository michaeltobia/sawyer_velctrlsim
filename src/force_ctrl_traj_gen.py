#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, TransformStamped, Vector3, Quaternion
from math import sin, cos, pi, sqrt

class TrajectoryGenerator:
    def __init__(self):
        # Init node
        rospy.init_node('ref_trajectory')
        # Init FT data subscriber
        rospy.Subscriber("/biased_ft_data", WrenchStamped, self.store_ft_sensor_dat)
        # Init desired_trajectory publisher
        self.pub_motion = rospy.Publisher('/desired_trajectory',
                            TransformStamped, queue_size=10)
        # Init desired trajectory frame TransformStamped message
        self.desired_trajectory_frame = TransformStamped()
        # Set child frame to simulated Sawyer base
        #   (tf prefix must be changed in vel_ctrl_traj .launch and .rviz
        #    if changed here)
        self.desired_trajectory_frame.child_frame_id = "right_l6"
        # Set desired trajectory frame ID
        self.desired_trajectory_frame.header.frame_id = "trajectory"

        # Set control gains
        # K: K_Fx, K_Fy, K_Fz, K_Tx, K_Ty, K_Tz
        self.K_p = [2,0,0,0,0,0]
        self.K_i = [0,0,0,0,0,0]
        self.K_d = [0,0,0,0,0,0]
        self.des_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cur_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.x_d_int_err = 0
        self.err_buff = [0,0,0]



    def trajectory(self, t):
        ## Trajectory is specified relative to base frame
        # Desired displacement 'p_d -> vel_ctrl'
        shitf_x_d = 0.5 # Will apply corrective force towards this position in task space
        # x_d > 0 --> away from controller tower
        # Calculate f_x error
        f_x_err = self.cur_wrench[2] - self.des_wrench[2]
        if f_x_err < 0.1:
            f_x_err = 0
        P_f_x = self.K_p[0]*0.05*(f_x_err)

        self.err_buff[0] = self.err_buff[1]
        self.err_buff[1] = f_x_err
        self.x_d_int_err += 0.05*np.sum(self.err_buff)*(1.0/120.0)
        I_f_x = self.K_i[0]*self.x_d_int_err
        D_f_x = self.K_d[0]*((self.err_buff[1]-self.err_buff[0])*(0.1/60))

        x_d = P_f_x + I_f_x + D_f_x + 0.5
        print self.des_wrench[2] - self.cur_wrench[2]
        if (x_d > 0.9):
            x_d = 0.9
        elif (x_d < 0.3):
            x_d = 0.3
        # y_d > 0 --> left when on control tower side facing Sawyer
        y_d = 0
        # z_d > 0 --> up
        z_d = 0.5
        p_d = [x_d, y_d, z_d]

        # Desired rotation 'Q_d -> vel_ctrl'
        # follows RHR
        theta = pi/2.0 # Desired rotation degrees
        q0_d = cos(theta/2.0) # Rotation magnitude (*not quaternion magnitude)
        q1_d = 0.0 * sin(theta/2.0) # Unit axis 'i' / base frame axis x
        q2_d = 1.0 * sin(theta/2.0) # Unit axis 'j' / base frame axis y
        q3_d = 0.0 * sin(theta/2.0) # Unit axis 'k' / base frame axis z
        Q_d = np.array([q0_d, q1_d, q2_d, q3_d]) # convert to np.array()
        Q_d = Q_d / np.linalg.norm(Q_d) # normalize Q_d
        Q_d = np.ndarray.tolist(Q_d) # convert back to list
        return p_d, Q_d

    def publish_trajectory(self):
        # Stamp desired trajectory TransformStamped message header
        self.desired_trajectory_frame.header.stamp = rospy.Time.now()
        # Get desired trajectory transform at current time
        p_d, Q_d = self.trajectory(rospy.get_time())
        # Apply desired trajectory to TransformStamped message
        self.desired_trajectory_frame.transform.translation = \
                                Vector3(p_d[0], p_d[1], p_d[2])
        self.desired_trajectory_frame.transform.rotation = \
                                Quaternion(Q_d[0], Q_d[1], Q_d[2], Q_d[3])
        # Publish TransformStamped desired trajectory message
        self.pub_motion.publish(self.desired_trajectory_frame)
        rate.sleep() # Sleep for global rate

    def store_ft_sensor_dat(self, data):
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        tx = data.wrench.torque.x
        ty = data.wrench.torque.y
        tz = data.wrench.torque.z
        self.cur_wrench = [fx, fy, fz, tx, ty, tz]
        # print self.cur_wrench


if __name__=='__main__':
    try:
        out = TrajectoryGenerator()
        rate = rospy.Rate(60) # Global rate 20Hz for sim
        while not rospy.is_shutdown():
            out.publish_trajectory()
    except rospy.ROSInterruptException:
        raise e
