#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
import time

 # Flag 0 : all stop
 # Flag 1 : coastal_nav & tracking & Lidar detection Until the end
 # Flag 2 : TSP on
 # Flag 3 : TSP off & WPT on (I give node where the USV have to go)
 # Flag 4 : WPT off // Camera detection & following & ReID on
 # Flag 5 : mode state ( mode 1: go flag 6, mode 2: go flag 2) 
 # Flag 6 : WPT on 
 # Flag 7 : WPT off // ship following & video streaming on for KSB confirmation (confirmation 0: go to flag 5, confirmation 1: go to Flag 8) 
 # Flag 8 : KSB confirmation is ok finish inspection phase.
    
class MissionPlanner:
    def __init__(self):
        self.flag = 0
        self.video = False
        self.timer_started = False
        self.start_time = 0.0

        self.publisher_flag = rospy.Publisher("usv_flag", Int32, queue_size=10)
        self.publisher_video = rospy.Publisher("toggle_switch", Bool, queue_size=10)

        rospy.Subscriber("flag_1", Int32, self.flag_1_callback)
        rospy.Subscriber("flag_2", Int32, self.flag_2_callback)
        rospy.Subscriber("flag_3", Int32, self.flag_3_callback)
        rospy.Subscriber("flag_4", Int32, self.flag_4_callback)
        rospy.Subscriber("flag_5", Int32, self.flag_5_callback)
        # rospy.Subscriber("flag_6", Int32, self.flag_6_callback)
        rospy.Subscriber("flag_7", Int32, self.flag_7_callback)
        rospy.Subscriber("ksb_confirmation", Bool, self.flag_8_callback)
        rospy.Subscriber("mode_state", Int32, self.mode_sate_callback)
        rospy.Subscriber("reset", Int32, self.reset_callback)

    def flag_1_callback(self, msg):
        if self.flag == 0 and msg.data == 1:
            self.flag = 1

    def flag_2_callback(self, msg):
        if self.flag == 1 and msg.data == 1:
            self.flag = 2

    def flag_3_callback(self, msg):
        if self.flag == 2 and msg.data == 1:
            self.flag = 3
        # else:
        #     self.flag = 2

    def flag_4_callback(self, msg):
        if self.flag == 3 and msg.data == 1:
            self.flag = 4
        # else:
        #     self.flag = 2

    def flag_5_callback(self, msg):
        # if self.flag == 4 and msg.data == 1:
        #     self.flag = 5
        # elif self.flag == 4 and msg.data == 0:
        #     self.flag = 2
        if self.flag == 4:
            self.flag = 5  

    def mode_sate_callback(self, msg):
        # if self.flag == 5 and msg.data == 1:
        #     self.flag =6
        # elif self.flag ==5 and msg.data ==0:
        #     self.flag =2
        if msg.data == 0:
            self.flag = 2
        elif msg.data == 1:
            self.flag = 6

    # def flag_6_callback(self, msg):
    #     if self.flag == 5 and msg.data == 1:
    #         self.flag = 6
    #     # else:
    #     #     self.flag =2

    def flag_7_callback(self, msg):
        if self.flag == 6 and msg.data == 1:
            self.flag = 7
            self.video = True
            self.start_time = rospy.get_time()
            self.timer_started = True
        # else:
        #     self.flag =2
            
    def flag_8_callback(self, msg):
        if self.flag == 7 and msg.data == True:
            self.flag = 8

        elif self.flag == 7 and msg.data == False:
            self.flag = 5
            self.video = False
            self.timer_started = False

    def reset_callback(self, msg):
        if msg.data == 1:
            self.flag = 0
            self.video = False

    def publish_flag(self):
        if self.timer_started and (rospy.get_time() - self.start_time) >= 120:
            self.flag = 5
            self.timer_started = False
            self.video = False
        msg = Int32()
        msg.data = self.flag
        self.publisher_flag.publish(msg)
        rospy.loginfo("Flag is {}".format(self.flag))
        if self.flag == 7:
            rospy.loginfo("Time : {}".format(rospy.get_time() - self.start_time))

    def publish_video(self):
        msg = Bool()
        msg.data = self.video
        self.publisher_video.publish(msg)
        rospy.loginfo("Video is {}".format(self.video))

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    mission_planner = MissionPlanner()
    rate = rospy.Rate(10)  # 10Hz
    try:
        while not rospy.is_shutdown():
            mission_planner.publish_flag()
            mission_planner.publish_video()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
