import sys
import os
import math
import rospy
from std_msgs.msg import Float32,Float64, Int16, Int8
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

def normal_distance(x, y, yaw):

    # if yaw < 0.000001 and yaw > 0:
    #     yaw = 0.000001
    # if yaw > -0.000001 and yaw < 0:
    #     yaw = -0.000001
    
    distance = (((x/math.tan(yaw))+ y))/math.sqrt((1/math.tan(yaw))**2+1)
    # distance = (abs((x/math.tan(yaw))+ y))/math.sqrt((1/math.tan(yaw))**2+1)
    if yaw < 0:
        distance = -distance
    
    return distance


def bound_length(x, y, yaw, boundary_angle):

    # if yaw < 0.000001 and yaw > 0:
    #     yaw = 0.000001
    # if yaw > -0.000001 and yaw < 0:
    #     yaw = -0.000001
    
    x_0 = ((x/math.tan(yaw))+ y)/(math.tan(yaw) + 1/math.tan(yaw))
    y_0 = math.tan(yaw) * x_0

    distance = math.sqrt((x_0 - x)**2 + (y_0 - y)**2)
    bound_distance = math.tan(boundary_angle) * distance

    return distance, bound_distance

def angular_transform(Target_x, Target_y,Target_yaw, USV_x, USV_y ,USV_yaw):
    rel_psi = Target_yaw - USV_yaw

    if rel_psi > 3.141592:
        rel_psi = rel_psi - 2*3.141592
    elif rel_psi < -3.141592:
        rel_psi = rel_psi + 2*3.141592


    rel_x = (Target_x - USV_x)*math.cos(USV_yaw) + (Target_y - USV_y)*math.sin(USV_yaw)

    rel_y = (Target_x - USV_x)*math.sin(-USV_yaw) + (Target_y - USV_y)*math.cos(-USV_yaw)
    
    return rel_x, rel_y, rel_psi

def lla2utm(curr_x, curr_y):
    dLatitude = curr_x
    dLongitude = curr_y

    dLat = dLatitude * np.pi/180
    dLon = dLongitude * np.pi/180

    lon0_f = np.floor(dLongitude/6)*6+3
    lon0 = lon0_f*np.pi/180
    k0 = 0.9996
    FE = 500000
    FN = (dLatitude < 0)*10000000

    Wa = 6378137
    We = 0.081819190842965
    WN = Wa/np.sqrt( 1 - np.power(We,2)*np.power(np.sin(dLat),2))
    WT = np.power(np.tan(dLat), 2)
    WC = (np.power(We,2)/(1-np.power(We,2)))*np.power(np.cos(dLat),2)
    WLA = (dLon - lon0)*np.cos(dLat)
    WM = (Wa*((1 - np.power(We,2)/4 - 3*np.power(We,4)/64 - 5*np.power(We,6)/256)*dLat - (3*np.power(We,2)/8 + 3*np.power(We,4)/32 + 45*np.power(We,6)/1024)*np.sin(2*dLat) + (15*np.power(We,4)/256 + 45*np.power(We,6)/1024)*np.sin(4*dLat) - (35*np.power(We,6)/3072)*np.sin(6*dLat)) )

    Weps = 0.006739496742333
    # Easting
    m_dUTM_X = (FE + k0*WN*(WLA + (1 - WT + WC)*np.power(WLA,3)/6	+ (5 - 18*WT + np.power(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120))
    # Northing
    m_dUTM_Y = (FN + k0*WM + k0*WN*np.tan(dLat)*(np.power(WLA,2)/2 + (5 - WT + 9*WC + 4*np.power(WC,2))*np.power(WLA,4)/24 + (61 - 58*WT + np.power(WT,2) + 600*WC - 330*Weps)*np.power(WLA,6)/720))
    # Zone
    # m_nUTM_Zone = np.int(np.floor(lon0_f/6)+31)

    return m_dUTM_X, m_dUTM_Y



class Control_Param():
    def __init__(self):

        self.switch = 2

        self.Target_vessel_pose_data = 0.0
        self.Target_vessel_pose_x = 0.0
        self.Target_vessel_pose_y = 0.0
        self.Target_vessel_pose_yaw = 0.0

        self.USV_pose_data = 0.0
        self.USV_pose_x = 0.0
        self.USV_pose_y = 0.0
        self.USV_pose_yaw = 0.0

        self.U = 0.0
        self.U_const = 300.0
        self.U_delta = 0.0
        self.U_N = 0.0
        self.T_con = 0.0
        self.T_con_max = 600
        self.thrust_right = 0.0
        self.thrust_left = 0.0
        self.joint_right = 0.0
        self.joint_left = 0.0


        self.dtime = 0.0
        self.del_time = 0.0
        self.before_time = 0.0
        self.con_time = 0.0
        self.dock_con_time = 0.0

        self.Y_Kp = 70.0
        self.Y_Kd = 20.0
        self.N_Kp = 500.0
        self.N_Kd = 200.0 
        self.X_Kp = 6.0
        self.X_Kd = 1.0     
        self.X_Kp_positive = 20.0
        self.X_Kp_negative = 20.0
        self.U_default = 70


        self.ds = 8
        self.ddp = 40
        self.rel_x = 0.0
        self.rel_y = 0.0
        self.rel_yaw = 0.0

        self.LOS = 0.0
        self.LOS_e = 0.0
        self.LOS_e_before = 0.0
        self.yaw_e = 0.0
        self.yaw_e_before = 0.0

        self.e_x = 0.0
        self.e_y = 0.0
        self.e_x_before = 0.0
        self.e_y_before = 0.0
        self.y_bias = 1
        self.threshold_length = 0.0

        self.distance = 20.0
        self.DP_point_x = 0.0
        self.DP_point_y = 0.0

        self.delta = 0.0
        self.delta_sp = math.pi*5/180
        self.delta_default = math.pi*5/180
        self.delta_default_n = math.pi*5/180

        self.desired_yaw = 0

        self.dock_switch = 0.0

        self.dock_distance = 20.0

        self.heading_search_distance = 100.0

        self.angle_e = 0.0

        self.wpt_Kp = 100
        self.wpt_Kd = 10

        self.delta_min = math.pi*10/180

        self.real_thrust_right = 0.0
        self.real_thrust_left = 0.0
        self.before_thrust_right = 0.0
        self.before_thrust_left = 0.0
        self.thrust_bound = 50.0


        self.rel_vx = 0.0
        self.rel_vy = 0.0
        self.rel_x_before = 0.0
        self.rel_y_before = 0.0

        self.K_rel = 10.0
        self.search_time = 0.0

        self.joint_rotation_time  = 0.0


    def Imu_pose_sub(self,data):
        quat = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w])
        self.USV_pose_yaw= quat[2] #+ 3.141592*0.2

        if self.USV_pose_yaw < 0:
            self.USV_pose_yaw += 2 * 3.141592

    def sub_gps(self,data):
        self.USV_pose_x = data.point.x
        self.USV_pose_y = data.point.y
        
            
    def Target_Imu_pose_sub(self,data):
        quat = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w])
        # mat = tf.Matrix3x3(quat)

        # r, p, y = mat.getRPY()
        # print(data.orientation.x)
        self.Target_vessel_pose_yaw = quat[2]

        if self.Target_vessel_pose_yaw < 0:
            self.Target_vessel_pose_yaw += 2 * 3.141592
        # print(self.Target_vessel_pose_yaw)


    def Target_sub_gps(self,data):
        zone_ = 52
        latitude = data.latitude
        longitude = data.longitude

        latitude = 36.592680
        longitude = 126.294228
       
        if latitude == 0.0 and longitude == 0.0:
            print("The GPS data is not valid | Case 0: Zero Values from GPS")
            self.Target_vessel_pose_x = 0.0
            self.Target_vessel_pose_y = 0.0
        elif longitude > ((zone_ * 6) - 180) or longitude < (((zone_ * 6) - 180) - 6):
            print("The GPS data is not valid | Case 1: Measurement out of UTM Zone")
            self.Target_vessel_pose_x = 0.0
            self.Target_vessel_pose_y = 0.0
        else:

            self.Target_vessel_pose_x, self.Target_vessel_pose_y = lla2utm(latitude, longitude)
    def dock(self,msg):
        self.dock_switch = msg.data
        

    def control_time(self,msg):
        self.nanosec = msg.header.stamp.nsecs * 0.000000001
        self.sec = msg.header.stamp.secs


        if self.before_time == 0.0:
            self.before_time = self.sec + self.nanosec

        self.del_time = abs(self.before_time - (self.sec + self.nanosec))
        self.dtime = self.dtime + self.del_time
        self.before_time = self.sec + self.nanosec

        if self.dtime > 0.1:
            if self.dock_switch == 2:
                # print(self.dtime)



        ########################################################################### N control ###########################################################################
                self.yaw_e = self.Target_vessel_pose_yaw - self.USV_pose_yaw

                if self.yaw_e > 3.141592:
                    self.yaw_e = self.yaw_e - 2*3.141592
                elif self.yaw_e < -3.141592:
                    self.yaw_e = self.yaw_e + 2*3.141592


                print("yaw_e : ", self.desired_yaw*180/math.pi)

                self.U_N = ( self.N_Kp*self.desired_yaw + self.N_Kd * (self.desired_yaw - self.yaw_e_before)*10)
                

                self.yaw_e_before = self.desired_yaw


        #################################################################################################################################################################

                self.rel_x, self.rel_y, self.rel_yaw = angular_transform(self.Target_vessel_pose_x, self.Target_vessel_pose_y,self.Target_vessel_pose_yaw, self.USV_pose_x, self.USV_pose_y ,self.USV_pose_yaw)



                ############################################################
                self.rel_vx = (self.rel_x - self.rel_x_before)*10
                self.rel_vy = (self.rel_y - self.rel_y_before)*10

                self.rel_x_before = self.rel_x
                self.rel_y_before = self.rel_y
                ############################################################



                self.e_y = normal_distance(self.rel_x, self.rel_y, self.rel_yaw)
                self.e_x , self.threshold_length = bound_length(self.rel_x, self.rel_y, self.rel_yaw, math.pi*20/180)
                self.e_y = self.e_y + self.y_bias
                self.e_x = self.e_x - self.ds




        ########################################################################## wpt control ##########################################################################
                if math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) > self.dock_distance:
                    self.angle_e = math.atan2(self.rel_y, self.rel_x)
                    
                    while self.angle_e < -math.pi and self.angle_e > math.pi:
                        if self.angle_e > math.pi:
                            self.angle_e = self.angle_e - 2*math.pi
                        elif self.angle_e < -math.pi:
                            self.angle_e = self.angle_e + 2*math.pi

                    self.U_N = self.wpt_Kp*math.atan2(self.rel_y, self.rel_x)
                    self.T_con = self.U_const


               

        #################################################################################################################################################################


        ########################################################################## 1st control ##########################################################################
                elif math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) < self.dock_distance:
                    if self.switch == 1:
                        # if self.e_x > 0:


                        self.delta_sp = math.pi*40/180
                        self.delta_default = math.pi*0/180
                        self.delta_default_n = math.pi*0/180


                        if (self.e_y < -self.threshold_length or self.e_y > self.threshold_length) and (self.e_x > 1 or self.e_x < -1):
                            self.switch = 2
                        elif self.e_x < 1 and self.e_x > -1 and (self.e_y > 1 or self.e_y < -1):
                            self.switch = 2


                        if self.e_x < 1 and self.e_x > -1:
                            self.delta_sp = math.pi*5/180
                            self.delta_default_n = math.pi*0/180                            


                        if self.e_x < 0:
                            self.X_Kp = -self.X_Kp_negative
                            self.desired_yaw = self.yaw_e - (self.delta_sp*abs(self.e_x)/(-self.ds + self.ddp) + self.delta_default_n)
                        elif self.e_x > 0:
                            self.X_Kp = self.X_Kp_positive
                            self.desired_yaw = self.yaw_e + (self.delta_sp*abs(self.e_x)/(-self.ds + self.ddp) + self.delta_default_n)
                        

                        if self.desired_yaw > math.pi*60/180:
                            self.desired_yaw = math.pi*60/180
                        elif self.desired_yaw < -math.pi*60/180:
                            self.desired_yaw = -math.pi*60/180




                        if self.e_x < 1 and self.e_x > -1:
                            if self.desired_yaw - self.yaw_e > self.delta_min:
                                self.desired_yaw = self.delta_min + self.yaw_e

                            elif self.desired_yaw - self.yaw_e < -self.delta_min:
                                self.desired_yaw = -self.delta_min + self.yaw_e



                        self.T_con = self.X_Kp*self.e_x + self.X_Kd*(self.e_x - self.e_x_before)*10 + self.U_default

                        if self.T_con > self.T_con_max:
                            self.T_con = self.T_con_max
                        elif self.T_con < -self.T_con_max:
                            self.T_con = -self.T_con_max


                        self.e_x_before = self.e_x

                        self.joint_right = 0.0
                        self.joint_left = 0.0


        #################################################################################################################################################################


        ########################################################################## 2nd control ##########################################################################
                    elif self.switch == 2:

                        self.desired_yaw = self.yaw_e
                        
                        if self.e_y > -1 and self.e_y < 1:
                            self.con_time = self.con_time + self.dtime
                        else:
                            self.con_time = 0.0

                        if self.con_time > 1 and self.e_x > 1:
                            self.switch = 1
                            self.con_time = 0.0
                        elif self.con_time > 3 and self.e_x < 1:
                            self.switch = 3
                            self.con_time = 0.0
                        else:
                            self.T_con = (self.Y_Kp*self.e_y + self.Y_Kd*(self.e_y - self.e_y_before)*10)
                            self.delta = 0.0

                            if self.T_con > self.T_con_max:
                                self.T_con = self.T_con_max
                            elif self.T_con < -self.T_con_max:
                                self.T_con = -self.T_con_max

                        if self.e_x < -0.5:
                            self.switch = 1



                        self.e_y_before = self.e_y


                        self.joint_right = 0.0
                        self.joint_left = 0.0


        #################################################################################################################################################################

                    
        ########################################################################## 3rd control ##########################################################################
                    elif self.switch == 3:

                        self.desired_yaw = self.yaw_e

                        if self.joint_rotation_time < 3:
                            self.joint_rotation_time = self.joint_rotation_time + self.dtime
                            self.joint_right = 0.5*math.pi
                            self.joint_left = 0.5*math.pi
                            self.T_con = 0.0
                            self.U_N = 0.0

                        else:
                            self.T_con = 300.0
                            self.U_N = 0.0

                        # self.joint_right = 0.25*math.pi
                        # self.joint_left = -0.25*math.pi







        #################################################################################################################################################################





                print("Switch : ", self.switch)
                print("Boundary : ", self.threshold_length, "con_time : ", self.con_time)
                print("e_x : ", self.e_x, ", e_y : ", self.e_y)
                print("rel_x : ", self.rel_x, ", rel_y : ", self.rel_y)
                print("U_N : ", self.U_N,", T_con : ", self.T_con)
                print(" thrust right : ", self.real_thrust_right, ", delta : ", self.delta*180/math.pi)
                print("Dock switch : ", self.dock_switch)


                self.thrust_right = self.T_con + self.U_N
                self.thrust_left = self.T_con - self.U_N

                # if self.switch == 3:
                #     self.thrust_right = self.T_con
                #     self.thrust_left = - self.T_con




                self.dtime = 0.0
        
            





def main():
    # settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    param = Control_Param()

    pub_lt = rospy.Publisher('/workshop_setup/pods/left', Int16, queue_size=10)
    pub_rt = rospy.Publisher('/workshop_setup/pods/right', Int16, queue_size=10)
    pub_la = rospy.Publisher('/workshop_setup/pod_steer/left_steer', Float32, queue_size=10)
    pub_ra = rospy.Publisher('/workshop_setup/pod_steer/right_steer', Float32, queue_size=10)
    


    rospy.Subscriber('/imu/data', Imu, param.Imu_pose_sub)
    rospy.Subscriber('/utm_pos', PointStamped, param.sub_gps)
    rospy.Subscriber('/gx5/imu/data', Imu, param.Target_Imu_pose_sub)
    rospy.Subscriber('/ublox_gps/fix', NavSatFix, param.Target_sub_gps)
    rospy.Subscriber('/Dock', Int8, param.dock)

    rospy.Subscriber('/imu/data', Imu, param.control_time)

    rate = rospy.Rate(10) # 10 Hz



    while not rospy.is_shutdown():
        Thrust_right = Int16()
        Angle_right = Float32()
        Thrust_left = Int16()
        Angle_left = Float32()
        
        if param.thrust_left - param.real_thrust_left > param.thrust_bound:
            param.real_thrust_left = param.real_thrust_left + param.thrust_bound
        elif param.thrust_left - param.real_thrust_left < -param.thrust_bound:
            param.real_thrust_left = param.real_thrust_left - param.thrust_bound
        else:
            param.real_thrust_left = param.thrust_left


        if param.thrust_right - param.real_thrust_right > param.thrust_bound:
            param.real_thrust_right = param.real_thrust_right + param.thrust_bound
        elif param.thrust_right - param.real_thrust_right < -param.thrust_bound:
            param.real_thrust_right = param.real_thrust_right - param.thrust_bound
        else:
            param.real_thrust_right = param.thrust_right


        Thrust_right.data = math.floor(param.thrust_right)
        Angle_right.data = param.joint_right*180/math.pi
        Thrust_left.data = math.floor(param.thrust_left)
        Angle_left.data = param.joint_left*180/math.pi
        # Thrust_right.data = 0.0
        # Angle_right.data = 0.0
        # Thrust_left.data = 0.0
        # Angle_left.data = 0.0             
        if param.dock_switch == 2:    
            pub_rt.publish(Thrust_right)
            pub_ra.publish(Angle_right)
            pub_lt.publish(Thrust_left)
            pub_la.publish(Angle_left)
    
        rate.sleep()





if __name__ == '__main__':
    main()








