#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def flag_publisher():
    pub = rospy.Publisher('Flag', Int32, queue_size=10)
    rospy.init_node('flag_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    Flag = Int32()
    Flag.data = 1

    hello_str = "Go Flag Once"
    rospy.loginfo(hello_str)
    pub.publish(Flag)
    rate.sleep()

if __name__ == '__main__':
    try:
        flag_publisher()
    except rospy.ROSInterruptException:
        pass

