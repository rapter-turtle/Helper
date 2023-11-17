
import sys
import os
import math
import rospy
from std_msgs.msg import Float32,Float64, Int16, Int8, Int32, Float32MultiArray
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3, PointStamped
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix
from core_msgs.msg import string_w_header
import tf
import numpy as np
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z

class Control_Param():
    def __init__(self):


        self.dtime = 0.0
        self.global_dtime = 0.0
        self.del_time = 0.0
        self.before_time = 0.0
        self.con_time = 0.0
        self.dock_con_time = 0.0

        self.Target_x = 0.0
        self.Target_y = 0.0
        self.Target_yaw = 0.0

        self.global_Target_x = 0.0
        self.global_Target_y = 0.0
        self.global_USV_x = 0.0
        self.global_USV_y = 0.0


        self.USV_x = 0.0
        self.USV_y = 0.0
        self.USV_yaw = 0.0

        self.usv_imu = Imu()

        self.rel_x = 0.0
        self.rel_y = 0.0
        
        self.ID = 1000
        

    def sub_target_ship(self,msg):
        self.Target_x = msg.transforms[6].transform.translation.x
        self.Target_y = msg.transforms[6].transform.translation.y
        self.Target_yaw = euler_from_quaternion(msg.transforms[6].transform.rotation.x,msg.transforms[6].transform.rotation.y,msg.transforms[6].transform.rotation.z,msg.transforms[6].transform.rotation.w)

    def sub_usv_position(self,msg):
        self.USV_x = msg.transforms[2].transform.translation.x
        self.USV_y = msg.transforms[2].transform.translation.y

    def sub_usv_heading(self,msg):
        self.USV_yaw = euler_from_quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.usv_imu = msg
    # rospy.Subscriber('/world/coast/pose/info', TFMessage, param.sub_target_ship)
    # rospy.Subscriber('/usv/pose_static', TFMessage, param.sub_usv_position)
    # rospy.Subscriber('/usv/imu/data', Imu, param.sub_usv_heading)

    def control_time(self,msg):
        self.nanosec = msg.header.stamp.nsecs * 0.000000001
        self.sec = msg.header.stamp.secs


        if self.before_time == 0.0:
            self.before_time = self.sec + self.nanosec

        self.del_time = abs(self.before_time - (self.sec + self.nanosec))
        self.dtime = self.dtime + self.del_time
        self.global_dtime += self.del_time
        self.before_time = self.sec + self.nanosec
        
        if self.dtime > 0.1:

            self.rel_x = self.Target_x - self.USV_x
            self.rel_y = self.Target_y - self.USV_y
                        
            self.dtime = 0.0

        if self.global_dtime > 2.5:

            self.global_Target_x = self.Target_x
            self.global_Target_y = self.Target_y
            self.global_USV_x = self.USV_x
            self.global_USV_y = self.USV_y

            self.global_dtime = 0.0


            





def main():
    # settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    param = Control_Param()

    pub_lidar_track = rospy.Publisher('/lidar_track', Float32MultiArray, queue_size=10)
    pub_global_track = rospy.Publisher('/global_track', Float32MultiArray, queue_size=10)
    pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)

    #rospy.Subscriber('/lidar_track', Float32MultiArray, param.sub_local_xy)


    rospy.Subscriber('/world/coast/pose/info', TFMessage, param.sub_target_ship)
    rospy.Subscriber('/usv/pose_static', TFMessage, param.sub_usv_position)
    rospy.Subscriber('/usv/imu/data', Imu, param.sub_usv_heading)

    rospy.Subscriber('/usv/imu/data', Imu, param.control_time)

    rate = rospy.Rate(10) # 10 Hz



    while not rospy.is_shutdown():
        lidar = Float32MultiArray()
        global_t = Float32MultiArray()
        imu_usv = Imu()

        
        lidar.data = [1.0, param.ID, param.rel_x,param.rel_y,0, 0, param.Target_yaw]
        global_t.data = [param.global_USV_x, param.global_USV_y, 1.0, param.ID, param.global_Target_x, param.global_Target_y, 0.0]
        imu_usv = param.usv_imu

        pub_lidar_track.publish(lidar)
        pub_global_track.publish(global_t)
        pub_imu.publish(imu_usv)
       




        rate.sleep()




if __name__ == '__main__':
    main()







