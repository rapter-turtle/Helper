#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def wp_publisher():
    pub = rospy.Publisher('WPs', Float32MultiArray, queue_size=10)
    rospy.init_node('wp_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = Float32MultiArray()
    msg.data = [0,0,100,0,100,200,0,200] # x,y,x,y ...


    while not rospy.is_shutdown():
        hello_str = "WPs Published"
        rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        wp_publisher()
    except rospy.ROSInterruptException:
        pass

