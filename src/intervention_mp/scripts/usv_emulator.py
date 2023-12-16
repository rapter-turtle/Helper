#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

class USVEmulator():
    def __init__(self):
        rospy.init_node('usv_emulator', anonymous=True)
        ##### Parameters #####
        # Mode
        self.mode = 1 # 0:USV emulator (for Drn on-land test) , 1: Only Perception emulator (for USV gazebo test)
        # GT Vessel length
        self.GT_VESSEL_LENGTH = 18
        # Stop thrusters duration
        self.STOP_TIME = 1
        # DP convergence duration
        self.DP_CONV_TIME = 10
        ##### end Parameters #####

        self.dp_ref = Float32MultiArray()
        self.curr_bias = 0
        
        # Subscribers
        rospy.Subscriber('/dp_ref', Float32MultiArray, self.send_dp_status)
        # Publishers
        if self.mode == 0:
            self.pub_dp_ref = rospy.Publisher('/dp_ref', Float32MultiArray, queue_size=10)
        self.pub_vessel_len = rospy.Publisher('/vessel_length', Float32, queue_size=10)

    def send_dp_status(self,data):
        rospy.loginfo("send_dp_status %f",data.data[0])

        # For on-land Test
        if self.mode == 0: # 0:USV emulator
            # Stop Thrusters
            if len(data.data) == 2 and data.data[0] == 0 and data.data[1] == 0:
                # Stop delay
                rospy.sleep(self.STOP_TIME)

                # Publish thrusters stopped msg 
                self.dp_ref.data = [0]
                self.pub_dp_ref.publish(self.dp_ref)

            elif len(data.data) == 2 and data.data[0] == 1:
                # DP convergence delay
                rospy.sleep(self.DP_CONV_TIME)

                # Publish DP convergence msg 
                self.dp_ref.data = [1]
                self.pub_dp_ref.publish(self.dp_ref)

        # Save current bias for perception target vessel length simulator
        if len(data.data) == 2 and data.data[0] == 1:
            self.curr_bias = data.data[1]

    def generate_vessel_len(self):
        rospy.loginfo("generate_vessel_len")
        l = self.GT_VESSEL_LENGTH

        # Generate Gaussian noise
        mean = 0
        std_dev = 0.1
        noise = np.random.normal(mean, std_dev)

        # add noise
        l = l + noise

        # reduce l by a factor as bias increases
        red_fac = 2-np.exp(math.fabs(self.curr_bias)/30) # 1: 96%, 2: 93%, 3: 89%, 4: 85%, 5: 81%
        l = l * red_fac

        vessel_len = Float32()
        vessel_len.data = l
        return vessel_len

    def main(self):
        rospy.loginfo("sanity check")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            vessel_len = self.generate_vessel_len()
            self.pub_vessel_len.publish(vessel_len)
            rate.sleep()

if __name__ == '__main__':
    try:
        emul = USVEmulator()
        emul.main()
    except rospy.ROSInterruptException:
        pass