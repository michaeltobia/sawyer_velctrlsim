#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Vector3
from sawyer_velctrlsim.srv import Bias

class FTDataBias:
    def __init__(self):
        # Init node
        rospy.init_node('ft_bias_node')

        # Unbiased FT sensor data subscriber
        self.sub = rospy.Subscriber("/netft_data", WrenchStamped, self.apply_bias)
        # Biased FT data sensor publisher
        self.pub = rospy.Publisher('/biased_ft_data', WrenchStamped, \
                                    queue_size=10)
        # Biasing service
        self.serv = rospy.Service('bias_ft_sensor', Bias, self.set_bias)

        self.bias = np.zeros(6)
        self.biased_data = WrenchStamped()
        self.rolling_mean_cont = np.zeros([6,6])


    def apply_bias(self, unbiased_dat):


        # Reorganize incoming data into np.array()
        in_Fx = unbiased_dat.wrench.force.x
        in_Fy = unbiased_dat.wrench.force.y
        in_Fz = unbiased_dat.wrench.force.z
        in_Tx = unbiased_dat.wrench.torque.x
        in_Ty = unbiased_dat.wrench.torque.y
        in_Tz = unbiased_dat.wrench.torque.z
        in_dat = np.array([in_Fx, in_Fy, in_Fz, in_Tx, in_Ty, in_Tz])

        # Apply bias
        out_dat = in_dat - self.bias

        # A for length rolling average to cut down on chatter maybe
        for i in range(5):
            self.rolling_mean_cont[i] = self.rolling_mean_cont[i+1]
        self.rolling_mean_cont[5] = out_dat
        out_dat = np.mean(self.rolling_mean_cont, axis=0)

        # # Clip small values to reduce idle noise and runaway
        # for i in range(len(out_dat)):
        #     if abs(out_dat[i]) < 0.1:
        #         out_dat[i] = 0

        # Sort biased data into biased_data message
        self.biased_data.wrench.force.x = out_dat[0]
        self.biased_data.wrench.force.y = out_dat[1]
        self.biased_data.wrench.force.z = out_dat[2]
        self.biased_data.wrench.torque.x = out_dat[3]
        self.biased_data.wrench.torque.y = out_dat[4]
        self.biased_data.wrench.torque.z = out_dat[5]


        # Stamp and publish biased data
        self.biased_data.header.stamp = rospy.Time.now()
        self.pub.publish(self.biased_data)

    def set_bias(self, junk):
        # Reorganize last biased_data message into np.array()
        self.bias[0] += self.biased_data.wrench.force.x
        self.bias[1] += self.biased_data.wrench.force.y
        self.bias[2] += self.biased_data.wrench.force.z
        self.bias[3] += self.biased_data.wrench.torque.x
        self.bias[4] += self.biased_data.wrench.torque.y
        self.bias[5] += self.biased_data.wrench.torque.z
        return (True)

if __name__=='__main__':
    try:
        FTDataBias()
        rospy.spin()
    except rospy.ROSInterruptException:
        raise e
