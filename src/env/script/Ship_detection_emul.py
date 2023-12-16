
import sys
import os
import math
import rospy
from std_msgs.msg import Float32,Float64, Int16, Int8, Int32, Float64MultiArray
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3, PointStamped
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix
from core_msgs.msg import string_w_header
import tf
import numpy as np


class Control_Param():
    def __init__(self):

        self.ship_id = 0
        self.ship_similarity = 0.0
        self.reid_array = []
        self.usv_flag = 0
        self.usv_flag_before = 0
        self.N = 0
        self.start_time = 0.0
 

    def sub_sim_arr(self,msg):
        self.reid_array = msg.data

    def sub_usv_flag(self,msg):
        self.usv_flag_before = self.usv_flag
        self.usv_flag = msg.data
        
    def sub_wp_id(self,msg):
        if self.usv_flag == 2 or self.usv_flag == 3:
            self.ship_id = msg.data
               



def main():
    # settings = saveTerminalSettings()
    rospy.init_node('ship_detection_emulator', anonymous=True)
    param = Control_Param()



    rospy.Subscriber('/reid/sim_arr', Float64MultiArray, param.sub_sim_arr)
    rospy.Subscriber('/usv_flag', Int32, param.sub_usv_flag)
    rospy.Subscriber('/wp_id', Float32, param.sub_wp_id)


    pub_trg_info = rospy.Publisher('/trg_info', Float64MultiArray, queue_size=10 )
    pub_flag_5 = rospy.Publisher('/flag_5', Int32, queue_size=10 )
 


    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        if param.usv_flag_before == 3 and param.usv_flag == 4:
            param.start_time = rospy.get_time()
        if param.usv_flag == 4 and param.usv_flag_before == 4:
            print("Time : ", rospy.get_time() - param.start_time)
            if (rospy.get_time() - param.start_time) >= 120:
                go = Int32()
                go.data = 1
                pub_flag_5.publish(go)


        if param.usv_flag == 4 or param.usv_flag == 5:
            trg_info = Float64MultiArray()

            param.ship_similarity = param.reid_array[param.N]
            trg_info.data = [param.ship_id, param.ship_similarity]

            pub_trg_info.publish(trg_info)
        
        
        if param.usv_flag_before  == 5 and param.usv_flag == 2:
            param.N += 1

        if param.N == len(param.reid_array):
            param.N = 0

   
 

        rate.sleep()


if __name__ == '__main__':
    main()







