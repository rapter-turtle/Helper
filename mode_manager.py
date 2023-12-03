#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32, Float64MultiArray

# When flag is 6, send highest similarity target ID 

class Mode_manager:
    def __init__(self):
        self.mode = 0
        self.id_similarity = []
        self.usv_flag= 0 
        self.before_usv_flag = 0
        
        self.publisher_HID = rospy.Publisher('wp_id',Float32,queue_size=10)
        self.publisher_mode = rospy.Publisher('mode_state',Int32,queue_size=10)

        rospy.Subscriber("trg_info", Float64MultiArray,self.trg_callback)
        rospy.Subscriber("usv_flag",Int32,self.usv_flag_callback)
        rospy.Subscriber("reset",Int32,self.reset_callback)


    def trg_callback(self, msg):
        if self.mode == 0:
            append = 0
            for i in range(len(self.id_similarity)):
                if self.id_similarity[i][0] == msg.data[0]:
                    append = 1
            if append == 0:
                self.id_similarity.append((msg.data[0], msg.data[1])) 
                self.id_similarity = sorted(self.id_similarity, key=lambda x: x[1], reverse=True)
                print(self.id_similarity)

    def usv_flag_callback(self, msg):
        self.usv_flag = msg.data

    def reset_callback(self, msg):
        self.usv_flag = msg.data

    def publish_wp_hid(self):
        if self.id_similarity:  # Check if the list is not empty
            highest_similarity_id = self.id_similarity[0][0]  # Assuming first element is the ID
            msg = Float32()
            msg.data = highest_similarity_id
            self.publisher_HID.publish(msg)
            rospy.loginfo("Published highest similarity ID: {}".format(highest_similarity_id))
        else:
            rospy.loginfo("No similarity data available to publish")
        
    def dropout(self):
        if self.before_usv_flag == 7 and self.usv_flag == 5:
            self.id_similarity = self.id_similarity[1:]
        self.before_usv_flag = self.usv_flag

    def publish_mode(self):
        threshold =3 
        if len(self.id_similarity) > threshold:
            self.mode = 1
     
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
