#!/usr/bin/env python3
### import ###
import rospy
from std_msgs.msg import Float64MultiArray  # ros std_msgs.msg Folat64MultiArray import
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Float64
import math

port = 0
starboard = 0

velocity = 0
K = 0
R_s = 0


##Inputpub##
def Parampub():
    pub_V = rospy.Publisher('/vel',Int16,queue_size=10) #Declare publisher to publish topic of which name is /input
    pub_K = rospy.Publisher('/gain',Float64,queue_size=10) #Declare publisher to publish topic of which name is /input
    pub_R_s = rospy.Publisher('/radius',Float64,queue_size=10) #Declare publisher to publish topic of which name is /input
    rospy.init_node('input_node', anonymous = True)
    rate = rospy.Rate(10) # 10Hz(0.1s)
    global x_d
    global y_d
    global psi_d
    while not rospy.is_shutdown():
        velocity = input("velocity : ")    #user command
        K = input("gain : ") # user command
        R_s = input("radius : ")
        data = Int16()
        data.data = int(velocity)
        pub_V.publish(data)
        data = Float64()
        data.data = float(K) 
        pub_K.publish(data)
        data = Float64()
        data.data = float(R_s)
        pub_R_s.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        Parampub()
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit() # ctrl+c --> terminate code
