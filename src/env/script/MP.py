
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


        self.usv_flag = 3
        self.flag_4 = 0
        self.flag_start = 0

    def sub_f4(self,msg):
        self.flag_4 = float(msg.data)
        # print(self.thrust_left)
    def sub_fs(self,msg):
        self.flag_start = float(msg.data)



def main():
    # settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    param = Control_Param()



    rospy.Subscriber('/flag_4', Int32, param.sub_f4)
    rospy.Subscriber('/flag_start', Int32, param.sub_fs)



    pub_flag = rospy.Publisher('/usv_flag', Int32,queue_size=10 )


    rate = rospy.Rate(10) # 10 Hz



    while not rospy.is_shutdown():
        flag = Int32()

        if param.flag_4 == 1 and param.usv_flag == 3:
            param.flag_4 = 0
            param.usv_flag = 4
        if param.flag_start == 1 and param.usv_flag == 4:
            param.flag_start = 0
            param.usv_flag = 3
        if param.flag_start == 2 and param.usv_flag == 4:
            param.flag_start = 0
            param.usv_flag = 6        
        if param.flag_start == 1 and param.usv_flag == 6:
            param.flag_start = 0
            param.usv_flag = 3   
    

        flag.data = param.usv_flag


        print("Flag: ",param.usv_flag)



        pub_flag.publish(flag)

   

        rate.sleep()




if __name__ == '__main__':
    main()







