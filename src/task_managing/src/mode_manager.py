#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32, Float64MultiArray, Float32MultiArray

# When flag is 6, send highest similarity target ID 

class Mode_manager:
    def __init__(self):
        self.mode = 0
        self.id_similarity = []
        self.usv_flag= 0 
        self.before_usv_flag = 0
        self.threshold = 0.2
        self.M = 4
        self.N = 2
        self.trg_num = 7
        self.global_track_list = []
        
        self.publisher_HID = rospy.Publisher('wp_id',Float32,queue_size=10)
        self.publisher_mode = rospy.Publisher('mode_state',Int32,queue_size=10)

        rospy.Subscriber("trg_info", Float64MultiArray,self.trg_callback)
        rospy.Subscriber("usv_flag",Int32,self.usv_flag_callback)
        rospy.Subscriber("global_track",Float32MultiArray,self.global_track_callback)
        rospy.Subscriber("reset",Int32,self.reset_callback)
        rospy.Subscriber("threshold",Float32,self.threshold_callback)
        rospy.Subscriber("M",Int32,self.M_callback)
        rospy.Subscriber("N",Int32,self.N_callback)


    def trg_callback(self, msg):
        if self.mode == 0:
            append = -1
            # print(int(len(self.id_similarity)))
            for i in range(int(len(self.id_similarity))):
                if self.id_similarity[i][0] == msg.data[0]:
                    append = i
            if append == -1:
                self.id_similarity.append([msg.data[0], msg.data[1]]) 
                self.id_similarity = sorted(self.id_similarity, key=lambda x: x[1], reverse=True)
                print(self.id_similarity)

            elif append > -1:
                if self.id_similarity[append][1] < msg.data[1]:
                    self.id_similarity[append][1] = msg.data[1] 
                    print(self.id_similarity)

    def usv_flag_callback(self, msg):
        self.usv_flag = msg.data

    def reset_callback(self, msg):
        if msg.data == 1:
            self.id_similarity = []
            self.mode = 0

    def global_track_callback(self, msg):
        self.trg_num = msg.data[2]
        self.global_track_list = msg.data
        # print(self.global_track_list )

    def threshold_callback(self, msg):
        self.threshold = msg.data
    def M_callback(self, msg):
        self.M = msg.data
    def N_callback(self, msg):
        self.N = msg.data

    def publish_wp_hid(self):
        if self.id_similarity:  # Check if the list is not empty
            highest_similarity_id = self.id_similarity[0][0]  # Assuming first element is the ID= =
            if self.usv_flag == 6:
                msg = Float32()
                msg.data = float(highest_similarity_id)
                self.publisher_HID.publish(msg)
                # rospy.loginfo("Published highest similarity ID: {}".format(highest_similarity_id))
        # else:
            # rospy.loginfo("No similarity data available to publish")
        
    def dropout(self):
        if self.before_usv_flag == 7 and self.usv_flag == 5:
            self.id_similarity = self.id_similarity[1:]
        self.before_usv_flag = self.usv_flag

        find = 0
        print(self.id_similarity)
        for i in range(int(self.trg_num)):
            if self.id_similarity[0][0] == self.global_track_list[3+5*i]:
                find = 1
            # print(self.global_track_list[3+5*i])
        if find == 0:
            self.id_similarity = self.id_similarity[1:]

    def publish_mode(self):
        if self.mode == 0:
            if len(self.id_similarity) < self.M:
                self.mode = 0
            elif len(self.id_similarity) >= self.M and len(self.id_similarity) < self.trg_num:
                if self.id_similarity[self.N-1][1] - self.id_similarity[self.N][1] > self.threshold:
                    self.mode = 1
                    print("Change mode to 1")
                else:
                    self.mode = 0
            elif len(self.id_similarity) == self.trg_num:
                self.mode = 1
                print("Change mode to 1")
        
        msg = Int32()
        msg.data = self.mode
        self.publisher_mode.publish(msg)

    # To do, we should decide when the mode is change to 1. 
        

if __name__ == '__main__':
    rospy.init_node('mode_manager')
    mode_manager = Mode_manager()
    rate = rospy.Rate(10)  # 10Hz
    try:
        while not rospy.is_shutdown():
            if mode_manager.usv_flag == 5:
                mode_manager.publish_mode()

            if mode_manager.usv_flag >= 5 and mode_manager.mode == 1:
                mode_manager.dropout()
                mode_manager.publish_wp_hid()
                
                # mode_manager.mode = 0
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
