
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



def normal_distance(relx, rely, relpsi):

    
    k = math.tan(0.5*math.pi - relpsi)

    y = (rely + k*relx)/(k*k+1)

    x = k*y

    e_x1 = (x)*math.cos(relpsi) + (y)*math.sin(relpsi)

    e_y1 = -(x)*math.sin(relpsi) + (y)*math.cos(relpsi)

    e_x0 = (relx)*math.cos(relpsi) + (rely)*math.sin(relpsi)

    e_y0 = -(relx)*math.sin(relpsi) + (rely)*math.cos(relpsi)

    e_y = e_y0

    e_x = e_x1


    return e_x


def bound_length(relx, rely, relpsi, boundary_angle,ds):

   
    k = math.tan(0.5*math.pi - relpsi)

    y = (rely + k*relx)/(k*k+1)

    x = k*y

    e_x1 = (x)*math.cos(relpsi) + (y)*math.sin(relpsi)

    e_y1 = -(x)*math.sin(relpsi) + (y)*math.cos(relpsi)

    e_x0 = (relx)*math.cos(relpsi) + (rely)*math.sin(relpsi)

    e_y0 = -(relx)*math.sin(relpsi) + (rely)*math.cos(relpsi)

    e_y = e_y0

    e_x = e_x1                               

    # distance = math.sqrt((x_0 - x)**2 + (y_0 - y)**2)
    bound_distance = abs(math.tan(boundary_angle) * (e_y + ds))

    return e_y, bound_distance

def angular_transform(Target_x, Target_y,Target_yaw, USV_x, USV_y ,USV_yaw):
    rel_psi = Target_yaw - USV_yaw
    # print(rel_psi)
    while rel_psi > 3.141592:
        rel_psi = rel_psi - 2*3.141592
    while rel_psi < -3.141592:
        rel_psi = rel_psi + 2*3.141592
    # print(rel_psi*180/math.pi)

    rel_x = (Target_x - USV_x)*math.cos(USV_yaw) + (Target_y - USV_y)*math.sin(USV_yaw)

    rel_y = (Target_x - USV_x)*math.sin(-USV_yaw) + (Target_y - USV_y)*math.cos(-USV_yaw)


    return rel_x, rel_y, rel_psi




class Control_Param():
    def __init__(self):

        self.switch = 1

        self.Target_vessel_pose_data = 0.0
        self.Target_vessel_pose_x = 0.0
        self.Target_vessel_pose_y = 0.0
        self.Target_vessel_pose_yaw = 180*math.pi/180
        
        self.USV_pose_data = 0.0
        self.USV_pose_x = 0.0
        self.USV_pose_y = 0.0
        self.USV_pose_yaw = 0.0

        self.U = 0.0
        self.U_const = 190.0
        self.U_delta = 0.0
        self.U_N = 0.0
        self.T_con = 0.0
        self.T_con_max = 10000000
        self.thrust_right = 0.0
        self.thrust_left = 0.0
        self.joint_right = 0.0
        self.joint_left = 0.0


        self.dtime = 0.0
        self.del_time = 0.0
        self.before_time = 0.0
        self.con_time = 0.0
        self.dock_con_time = 0.0

        self.Y_Kp = 50.0
        self.Y_Kd = 200.0
        self.N_Kp = 1000.0
        self.N_Kd = 5000.0 
        self.X_Kp = 10.0
        self.X_Kd = 120.0     
        self.U_default = 70
        self.K_beta = 0.001
        self.lamda = 10

        self.ds = 20
        self.ddp = 50
        self.rel_x = 0.01
        self.rel_y = 0.01
        self.rel_yaw = 0.01

        self.yaw_e = 0.0
        self.yaw_e_before = 0.0

        self.e_x = 0.0
        self.e_y = 0.0
        self.e_x_before = 0.0
        self.e_y_before = 0.0
        self.y_bias = 0
        self.threshold_length = 0.0

        self.delta = 0.0
        self.delta_sp = math.pi*20/180
        self.delta_default = math.pi*100/180
        self.delta_default_n = math.pi*20/180

        self.desired_yaw = 0

        self.dock_switch = 1
        self.before_dock_switch = 0

        self.dock_distance = 25.0
        self.avoid_distance = 35.0

        self.wpt_distance = 30.0 #############

        self.angle_e = 0.0

        self.wpt_Kp = 1000
        self.wpt_Kd = 100

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

        self.heading = 0.0
        self.heading_before = 0.0

        self.K_rel = 50.0
        self.search_time = 0.0

        self.found_heading = 0

        self.filtering_time = 0.0
        self.filtering_thres = 1.0  
        self.filtering_swithch = 0      

        self.global_vel_x = 0.0
        self.global_vel_y = 0.0

        self.real_rel_x = 0.0
        self.real_rel_y = 0.0

        self.rotate_thrust = 400

        self.target_dtime = 0.0
        self.heading_search_time = 0.0

        self.before_rel_yaw = 0.0

        self.lidar_heading = 0.0


        self.wpt_bound = 7
        self.wpt_x_avoidance = 0.0
        self.wpt_y_avoidance = 0.0

        self.ID = 0
        self.delta_max = 70*math.pi/180
        self.steer = 0*math.pi/180

        self.filtered_data_ex = []
        self.filtered_data_ey = []
        


    def Imu_pose_sub(self,data):
        quat = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w])
        self.USV_pose_yaw= quat[2] #+ 3.141592*0.2

        if self.USV_pose_yaw < -math.pi:
            self.USV_pose_yaw += 2 * 3.141592
        if self.USV_pose_yaw > math.pi:
            self.USV_pose_yaw -= 2 * 3.141592

      

    def sub_local_xy(self,msg):
        if len(msg.data) > 3:

               
            N = int(msg.data[0])
            # print(msg.data)

            for i in range(N):
                if self.ID == int(msg.data[1+8*i]):
                    self.real_rel_x = msg.data[1+8*i+1]
                    self.real_rel_y = msg.data[1+8*i+2]
                    # global_lidar_heading = -msg.data[1+6*i+5]  ############# definitly change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    global_lidar_heading = msg.data[1+8*i+5]
                    ################################################################## -- heading_on
                    # global_lidar_heading = self.Target_vessel_pose_yaw
                    ##################################################################
                    self.lidar_heading = global_lidar_heading - self.USV_pose_yaw

                    while self.lidar_heading < -math.pi:
                        self.lidar_heading += 2 * 3.141592
                    while self.lidar_heading > math.pi:
                        self.lidar_heading -= 2 * 3.141592
                    
                    

            # print(self.rel_x)
            self.filtering_swithch = 0
            self.filtering_time = 0.0
        else:
            if self.filtering_time < self.filtering_thres:
                self.filtering_time += self.dtime
            else:
                self.filtering_swithch = 1


    def sub_wp_ID(self, msg):
        self.ID = msg.data


    def dock(self,msg):
        self.dock_switch = msg.data
        if self.dock_switch == 2 and self.before_dock_switch >= 4:
            self.found_heading = 0

        if self.dock_switch == 5 and self.before_dock_switch >= 6:
            self.found_heading = 0


        self.before_dock_switch = self.dock_switch


    def WPT(self,data):
        self.wpt_Kp = data.data
    def sub_U_con(self,data):
        self.U_const = data.data
        # print(self.U_const)
    def sub_nkp(self,data):
        self.N_Kp = data.data
    def sub_nkd(self,data):
        self.N_Kd = data.data
    def sub_ykp(self,data):
        self.Y_Kp = data.data
    def sub_ykd(self,data):
        self.Y_Kd = data.data
    def sub_xkp(self,data):
        self.X_Kp = data.data
    def sub_xkd(self,data):
        self.X_Kd = data.data
    def sub_Tmax(self,data):
        self.T_con_max = data.data
    def sub_Target_heading(self, data):
        self.Target_vessel_pose_yaw = data.data
    def sub_K_beta(self,data):
        self.K_beta = data.data
    def sub_lamda(self,data):
        self.lamda = data.data        

 

            
    def control_time(self,msg):
        self.nanosec = msg.header.stamp.nsecs * 0.000000001
        self.sec = msg.header.stamp.secs


        if self.before_time == 0.0:
            self.before_time = self.sec + self.nanosec

        self.del_time = abs(self.before_time - (self.sec + self.nanosec))
        self.dtime = self.dtime + self.del_time
        self.before_time = self.sec + self.nanosec

        
        if self.dtime > 0.1:
            # print("a")
            if self.dock_switch == 4 or self.dock_switch == 5 or self.dock_switch == 7 or self.dock_switch == 8 :
                # print(self.dtime)
                        
                if self.found_heading >= 1:
                
                    self.rel_yaw = self.lidar_heading
                
                    ################################################################## -- heading_on
                    rel_error = self.rel_yaw - self.before_rel_yaw

                    if rel_error > 0.5*math.pi or rel_error < -0.5*math.pi:
                        self.rel_yaw += math.pi

                    while self.rel_yaw > math.pi:
                        self.rel_yaw -= 2*math.pi
                    while self.rel_yaw < -math.pi:
                        self.rel_yaw += 2*math.pi
                    ##################################################################

                    self.yaw_e = self.rel_yaw

                    self.before_rel_yaw = self.rel_yaw

                    # print(self.yaw_e)
                    # print("aaaaaaaaaaaaaaaaaaaaaaa")


        ########################################################################### N control ###########################################################################


                self.U_N = ( self.N_Kp*self.desired_yaw + self.N_Kd * (self.desired_yaw - self.yaw_e_before)*10)
                

                self.yaw_e_before = self.desired_yaw


        ######################################################################## Axis calculation #########################################################################

                # self.rel_x, self.rel_y, self.rel_yaw = angular_transform(self.rel_x, self.rel_y,self.Target_vessel_pose_yaw, 0.0, 0.0 ,self.USV_pose_yaw)
                rx =self.real_rel_x
                ry =self.real_rel_y

                self.rel_x = rx*math.cos(self.USV_pose_yaw) + ry*math.sin(self.USV_pose_yaw)

                self.rel_y = rx*math.sin(-self.USV_pose_yaw) + ry*math.cos(-self.USV_pose_yaw)
                

                self.e_y = normal_distance(self.rel_x, self.rel_y, self.rel_yaw)
                self.e_x , self.threshold_length = bound_length(self.rel_x, self.rel_y, self.rel_yaw, math.pi*30/180, self.ds)


                ######################################### Low Pass Filter ######################################### 



                ###################################################################################################

                self.e_y = (self.e_y + self.y_bias)
                self.e_x = self.e_x + self.ds
                # print("not inside boundary")

        ######################################################################## heading search #########################################################################
            
                if self.found_heading == 0:
                    ###### ooo #####
                    self.U_N = 0.0
                    self.T_con = 0.0


                    self.heading_search_time += self.dtime
                    self.target_dtime += self.dtime 

                    if self.heading_search_time <= 1.1:
                        print("------------- Search Heading -------------")
                        if self.target_dtime > 1.0:

                            #################################################################################################### -- heading_on
                            # if self.rel_x != self.rel_x_before and self.rel_y != self.rel_y_before:
                            #     self.rel_vx = (self.rel_x - self.rel_x_before)
                            #     self.rel_vy = (self.rel_y - self.rel_y_before)

                            #     self.rel_x_before = self.rel_x
                            #     self.rel_y_before = self.rel_y

                            #     self.heading = math.atan2(self.rel_vy, self.rel_vx)

                            #     while self.lidar_heading > math.pi:
                            #         self.lidar_heading -= 2*math.pi
                            #     while self.lidar_heading < -math.pi:
                            #         self.lidar_heading += 2*math.pi

                            #     while self.heading > math.pi:
                            #         self.heading -= 2*math.pi
                            #     while self.heading < -math.pi:
                            #         self.heading += 2*math.pi


                            #     heading_error = self.heading - self.lidar_heading

                            #     while heading_error > math.pi:
                            #         heading_error -= 2*math.pi
                            #     while heading_error < -math.pi:
                            #         heading_error += 2*math.pi

                            #     if heading_error > 0.5*math.pi or heading_error < -0.5*math.pi:
                            #         self.lidar_heading += math.pi

                            #     while self.lidar_heading > math.pi:
                            #         self.lidar_heading -= 2*math.pi
                            #     while self.lidar_heading < -math.pi:
                            #         self.lidar_heading += 2*math.pi
                                
                            #     new_heading_error = self.heading - self.lidar_heading
                                ####################################################################################################3

                                if self.lidar_heading > 0.5*math.pi or self.lidar_heading < -0.5*math.pi:
                                    self.lidar_heading += math.pi

                                while self.lidar_heading > math.pi:
                                    self.lidar_heading -= 2*math.pi
                                while self.lidar_heading < -math.pi:
                                    self.lidar_heading += 2*math.pi

                                

                                print("#####################################################################################")
                                print("Lidar : ", self.lidar_heading*180/math.pi)
                                # print("Error : ", new_heading_error*180/math.pi)
                                print(self.heading_search_time)
                                print("#####################################################################################")

                                self.rel_yaw = self.lidar_heading
                                self.yaw_e = self.rel_yaw
                                self.before_rel_yaw = self.rel_yaw

                                self.target_dtime = 0.0
                    else:
                        self.found_heading = 1
                        self.heading_search_time = 0.0
                        self.target_dtime = 0.0
                    ###### ooo #####

        #################################################################### Avoidance control ###########################################################################

                elif self.found_heading == 1:

                    if self.rel_yaw < 0.0 and self.rel_yaw > -160*math.pi/180:

                        self.wpt_x_avoidance = (self.rel_x + (self.avoid_distance-1)*math.cos(self.rel_yaw - math.pi))
                        self.wpt_y_avoidance = (self.rel_y + (self.avoid_distance)*math.sin(self.rel_yaw - math.pi))

                        self.angle_e = math.atan2(self.wpt_y_avoidance, self.wpt_x_avoidance)
                        
                        while self.angle_e < -math.pi or self.angle_e > math.pi:
                            if self.angle_e > math.pi:
                                self.angle_e = self.angle_e - 2*math.pi
                            elif self.angle_e < -math.pi:
                                self.angle_e = self.angle_e + 2*math.pi

                        self.U_N = self.wpt_Kp*self.angle_e
                        self.T_con = self.U_const

                        self.joint_right = self.steer
                        self.joint_left = self.steer

                        print("################## Avoidance ##################")
                        print('Ship ID : ', self.ID)
                        print("Found_heading : ", self.found_heading)
                        print("wpt_x" , self.wpt_x_avoidance)
                        print("wpt_y" , self.wpt_y_avoidance)
                        print("angle : ", self.angle_e*180/math.pi)
                        print("rel : ", self.rel_yaw)
                        print("U_N : ", self.U_N,", T_con : ", self.T_con)
                        # print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)

                        if math.sqrt(self.wpt_x_avoidance*self.wpt_x_avoidance + self.wpt_y_avoidance*self.wpt_y_avoidance) < self.wpt_bound:
                            self.found_heading = 2
                    else:
                        self.found_heading = 2


        ########################################################################## wpt control ##########################################################################
                elif math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) > self.wpt_distance and self.found_heading == 2:

                    wpt_x = (self.rel_x + (self.dock_distance-1)*math.cos(self.rel_yaw - math.pi*1.5))
                    wpt_y = (self.rel_y + (self.dock_distance)*math.sin(self.rel_yaw - math.pi*1.5))

                    self.angle_e = math.atan2(wpt_y, wpt_x)
                    
                    while self.angle_e < -math.pi or self.angle_e > math.pi:
                        if self.angle_e > math.pi:
                            self.angle_e = self.angle_e - 2*math.pi
                        elif self.angle_e < -math.pi:
                            self.angle_e = self.angle_e + 2*math.pi

                    self.U_N = self.wpt_Kp*self.angle_e
                    self.T_con = self.U_const

                    self.joint_right = self.steer
                    self.joint_left = self.steer

                    print("################## WPT ##################")
                    print('Ship ID : ', self.ID)
                    print("Found_heading : ", self.found_heading)
                    print("wpt_x" , wpt_x)
                    print("wpt_y" , wpt_y)
                    print("angle : ", self.angle_e*180/math.pi)
                    print("rel yaw : ", self.rel_yaw*180/math.pi)
                    print("U_N : ", self.U_N,", T_con : ", self.T_con)
                    # print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)

            
        #################################################################################################################################################################


        ########################################################################## 1st control ##########################################################################
                elif math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) < self.wpt_distance and self.found_heading ==2:
                    if self.switch == 1:
                        # if self.e_x > 0:


                        if self.e_y < -abs(self.threshold_length) or self.e_y > abs(self.threshold_length):
                            self.switch = 2                   


                        if self.e_x < 0:
                            change_e = (self.delta_sp*abs(self.e_x) - self.delta_default_n)
                            if change_e > self.delta_max:
                                change_e = self.delta_max
                            elif change_e < -self.delta_max:
                                change_e = -self.delta_max
                            self.desired_yaw = self.yaw_e + change_e
                            
                        if self.e_x >= 0:
                            change_e = (self.delta_sp*abs(self.e_x) + self.delta_default_n)
                            if change_e > self.delta_max:
                                change_e = self.delta_max
                            elif change_e < -self.delta_max:
                                change_e = -self.delta_max                            
                            self.desired_yaw = self.yaw_e + change_e


                        
                        if self.e_x < 2 and self.e_x > -2:
                            if self.desired_yaw - self.yaw_e > self.delta_min:
                                self.desired_yaw = self.delta_min + self.yaw_e

                            elif self.desired_yaw - self.yaw_e < -self.delta_min:
                                self.desired_yaw = -self.delta_min + self.yaw_e


                        if abs(self.desired_yaw) > math.pi*18000/180:
                            self.T_con = 0 
                        else:
                            self.T_con = abs(self.X_Kp*self.e_x + self.X_Kd*(self.e_x - self.e_x_before)*10 + self.U_default)

                            if self.T_con > self.T_con_max:
                                self.T_con = self.T_con_max
                            elif self.T_con < -self.T_con_max:
                                self.T_con = -self.T_con_max


                        self.e_x_before = self.e_x

                        self.joint_right = self.steer
                        self.joint_left = self.steer


        #################################################################################################################################################################


        ########################################################################## 2nd control ##########################################################################
                    elif self.switch == 2:

                        self.desired_yaw = self.yaw_e
                        
                        if self.e_y > -1.5 and self.e_y < 1.5:
                            self.con_time = self.con_time + self.dtime
                        else:
                            self.con_time = 0.0

                        if self.con_time > 1:
                            if self.e_x > 2.5 or self.e_x < -2.5:
                                self.switch = 1
                                self.con_time = 0.0


                        else:

                            if abs(self.desired_yaw) > math.pi*20/180:
                                self.T_con = 0
                            else:                               
                                ########################################################### SMC #####################################################################
                                s = self.e_y #+ self.lamda*(self.e_y - self.e_y_before)*10
                                #####################################################################################################################################
                                self.T_con = (self.Y_Kp*self.e_y + self.Y_Kd*(self.e_y - self.e_y_before)*10) + (1.0 / self.K_beta) * np.sign(s)
                                self.delta = 0.0

                                if self.T_con > self.T_con_max:
                                    self.T_con = self.T_con_max
                                elif self.T_con < -self.T_con_max:
                                    self.T_con = -self.T_con_max


                                
                        self.e_y_before = self.e_y


                        self.joint_right = self.steer
                        self.joint_left = self.steer

                        
        #################################################################################################################################################################

                    print("#########################################################################################")                    
                    print("Switch : ", self.switch)
                    print("Boundary : ", self.threshold_length, "con_time : ", self.con_time)
                    print("e_x : ", self.e_x, ", e_y : ", self.e_y)
                    print("rel_x : ", self.rel_x, ", rel_y : ", self.rel_y, "rel_psi : ",self.rel_yaw*180/math.pi)
                    print("U_N : ", self.U_N,", T_con : ", self.T_con)
                    print("filter time : ", self.filtering_time, ", filtering switch : ", self.filtering_swithch)
                    print("USV H : ", self.USV_pose_yaw*180/math.pi)
                    print("Lidar heading : ", self.lidar_heading*180/math.pi)
                    print("desired_yaw : ", self.desired_yaw*180/math.pi)
                    

                if self.T_con < 0.0:
                    F_T = -math.sqrt(((-self.T_con)**1.351)/0.0013)
                else:
                    F_T = math.sqrt(((self.T_con)**1.351)/0.0013)

                if self.U_N < 0.0:
                    U_N = -math.sqrt(((-self.U_N)**1.351)/0.0013)
                else:
                    U_N = math.sqrt(((self.U_N)**1.351)/0.0013)


                self.thrust_right = F_T + U_N
                self.thrust_left = F_T - U_N

                print("RPM : U_N : ", U_N,", T_con : ", F_T)
                print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)


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
    rospy.Subscriber('/usv_flag', Int32, param.dock)

    rospy.Subscriber('/imu/data', Imu, param.control_time)

    rospy.Subscriber('/Y_Kp' , Float64, param.sub_ykp)
    rospy.Subscriber('/Y_Kd' , Float64, param.sub_ykd)
    rospy.Subscriber('/X_Kp' , Float64, param.sub_xkp)
    rospy.Subscriber('/X_Kd' , Float64, param.sub_xkd)
    rospy.Subscriber('/N_Kp' , Float64, param.sub_nkp)
    rospy.Subscriber('/N_Kd' , Float64, param.sub_nkd)
    rospy.Subscriber('/T_max' , Float64, param.sub_Tmax)
    rospy.Subscriber('/wpt' , Float64, param.WPT)
    rospy.Subscriber('/U_con' , Float64, param.sub_U_con)
    rospy.Subscriber('/K_beta' , Float64, param.sub_K_beta)
    rospy.Subscriber('/lamda' , Float64, param.sub_lamda) 


    rospy.Subscriber('/lidar_track', Float32MultiArray, param.sub_local_xy)
    rospy.Subscriber('/wp_id', Float32, param.sub_wp_ID)
    rospy.Subscriber('/target_heading', Float64, param.sub_Target_heading)



    rate = rospy.Rate(10) # 10 Hz



    while not rospy.is_shutdown():
        Thrust_right = Int16()
        Angle_right = Float32()
        Thrust_left = Int16()
        Angle_left = Float32()

        rotate_Thrust_right = Int16()
        rotate_Angle_right = Float32()
        rotate_Thrust_left = Int16()
        rotate_Angle_left = Float32()

        rotate_Thrust_right.data = 400
        rotate_Angle_right.data = 0.0
        rotate_Thrust_left.data = -400
        rotate_Angle_left.data = 0.0

        
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

        if param.real_thrust_left > 1900:
            param.real_thrust_left = 1900
        if param.real_thrust_left < -1900:
            param.real_thrust_left = -1900
        if param.real_thrust_right > 1900:
            param.real_thrust_right = 1900
        if param.real_thrust_right < -1900:
            param.real_thrust_right = -1900


        Thrust_right.data = math.floor(param.real_thrust_right)
        Angle_right.data = param.joint_right*180/math.pi
        Thrust_left.data = math.floor(param.real_thrust_left)
        Angle_left.data = param.joint_left*180/math.pi

        if param.dock_switch == 4 or param.dock_switch == 5 or param.dock_switch == 7 or param.dock_switch == 8:
            if param.filtering_swithch == 0:
                pub_rt.publish(Thrust_right)
                pub_ra.publish(Angle_right)
                pub_lt.publish(Thrust_left)
                pub_la.publish(Angle_left)

            if param.filtering_swithch == 1:
                pub_rt.publish(rotate_Thrust_right)
                pub_ra.publish(rotate_Angle_right)
                pub_lt.publish(rotate_Thrust_left)
                pub_la.publish(rotate_Angle_left)                
 

        rate.sleep()




if __name__ == '__main__':
    main()







