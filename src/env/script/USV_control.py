
import sys
import os
import math
import rospy
from std_msgs.msg import Float32,Float64, Int16, Int8, Int32
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3, PointStamped
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix
from core_msgs.msg import string_w_header
import tf
import numpy as np


class Control_Param():
    def __init__(self):


        self.dtime = 0.0
        self.del_time = 0.0
        self.before_time = 0.0
        self.con_time = 0.0
        self.dock_con_time = 0.0

        self.thrust_right = 0
        self.thrust_left = 0
        self.thrust_steer_right = 0.0
        self.thrust_steer_left = 0.0



        self.thrust_bound = 50
        self.min_thrust = 200
        self.max_thrust = 2000

        self.real_thrust_left = 0.0
        self.real_thrust_right = 0.0
        self.pub_real_thrust_left = 0.0
        self.pub_real_thrust_right = 0.0

    def sub_thrust_left(self,msg):
        self.thrust_left = float(msg.data)
        # print(self.thrust_left)
    def sub_thrust_right(self,msg):
        self.thrust_right = float(msg.data)
    def sub_thrust_steer_left(self,msg):
        self.thrust_steer_left = msg.data
    def sub_thrust_steer_right(self,msg):
        self.thrust_steer_right = msg.data


    def control_time(self,msg):
        self.nanosec = msg.header.stamp.nsecs * 0.000000001
        self.sec = msg.header.stamp.secs


        if self.before_time == 0.0:
            self.before_time = self.sec + self.nanosec

        self.del_time = abs(self.before_time - (self.sec + self.nanosec))
        self.dtime = self.dtime + self.del_time
        self.before_time = self.sec + self.nanosec
        # print('a',self.dtime)

        if self.dtime > 0.1:

        
            if self.thrust_left - self.real_thrust_left > self.thrust_bound:
                self.real_thrust_left = self.real_thrust_left + self.thrust_bound
            elif self.thrust_left - self.real_thrust_left < -self.thrust_bound:
                self.real_thrust_left = self.real_thrust_left - self.thrust_bound
            else:
                self.real_thrust_left = self.thrust_left

            

            if self.thrust_right - self.real_thrust_right > self.thrust_bound:
                self.real_thrust_right = self.real_thrust_right + self.thrust_bound
            elif self.thrust_right - self.real_thrust_right < -self.thrust_bound:
                self.real_thrust_right = self.real_thrust_right - self.thrust_bound
            else:
                self.real_thrust_right = self.thrust_right


            if self.real_thrust_left > self.max_thrust:
                self.real_thrust_left = self.max_thrust
            if self.real_thrust_right > self.max_thrust:
                self.real_thrust_right = self.max_thrust

            if self.real_thrust_left < -self.max_thrust:
                self.real_thrust_left = -self.max_thrust
            if self.real_thrust_right < -self.max_thrust:
                self.real_thrust_right = -self.max_thrust

            

            if self.real_thrust_left < self.min_thrust and self.real_thrust_left > -self.min_thrust :
                self.pub_real_thrust_left = 0.0
            else:
                self.pub_real_thrust_left = self.real_thrust_left

            if self.real_thrust_right < self.min_thrust and self.real_thrust_right > -self.min_thrust :
                self.pub_real_thrust_right = 0.0
            else:
                self.pub_real_thrust_right = self.real_thrust_right                            
            

            self.dtime = 0.0

            





def main():
    # settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    param = Control_Param()

    # pub_lt = rospy.Publisher('/workshop_setup/pods/left', Int16, queue_size=10)
    # pub_rt = rospy.Publisher('/workshop_setup/pods/right', Int16, queue_size=10)
    # pub_la = rospy.Publisher('/workshop_setup/pod_steer/left_steer', Float32, queue_size=10)
    # pub_ra = rospy.Publisher('/workshop_setup/pod_steer/right_steer', Float32, queue_size=10)
    


    # rospy.Subscriber('/usv/left/thrust/cmd_thrust', Float64, param.sub_thrust_left)
    # rospy.Subscriber('/usv/right/thrust/cmd_thrust', Float64, param.sub_thrust_right)
    # rospy.Subscriber('/usv/left/thrust/joint/cmd_pos', Float64, param.sub_thrust_steer_left)
    # rospy.Subscriber('/usv/right/thrust/joint/cmd_pos', Float64, param.sub_thrust_steer_right)
    rospy.Subscriber('/usv/imu/data', Imu, param.control_time)


    rospy.Subscriber('/workshop_setup/pods/left', Int16, param.sub_thrust_left)
    rospy.Subscriber('/workshop_setup/pods/right', Int16, param.sub_thrust_right)
    rospy.Subscriber('/workshop_setup/pod_steer/left_steer', Float32, param.sub_thrust_steer_left)
    rospy.Subscriber('/workshop_setup/pod_steer/right_steer', Float32, param.sub_thrust_steer_right)
    


    pub_lt = rospy.Publisher('/usv/left/thrust/cmd_thrust', Float64,queue_size=10 )
    pub_rt = rospy.Publisher('/usv/right/thrust/cmd_thrust', Float64,queue_size=10 )
    pub_la = rospy.Publisher('/usv/left/thrust/joint/cmd_pos', Float64,queue_size=10 )
    pub_ra = rospy.Publisher('/usv/right/thrust/joint/cmd_pos', Float64,queue_size=10 )

    rate = rospy.Rate(10) # 10 Hz



    while not rospy.is_shutdown():
        Thrust_right = Float64()
        Angle_right = Float64()
        Thrust_left = Float64()
        Angle_left = Float64()
        



        Thrust_right.data = param.pub_real_thrust_right
        Angle_right.data = -param.thrust_steer_right*math.pi/180
        Thrust_left.data = param.pub_real_thrust_left
        Angle_left.data = -param.thrust_steer_left*math.pi/180

        print("l thrust: ",param.pub_real_thrust_left)
        print("r thrust: ",param.pub_real_thrust_right)
        print("l joint: ",param.thrust_steer_left)
        print("r joint: ",param.thrust_steer_right)


        pub_rt.publish(Thrust_right)
        pub_ra.publish(Angle_right)
        pub_lt.publish(Thrust_left)
        pub_la.publish(Angle_left)
   
 

        rate.sleep()




if __name__ == '__main__':
    main()







