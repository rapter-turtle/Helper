#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32

class IntervetionMP():
    def __init__(self):
        rospy.init_node('intervention_mp', anonymous=True)
        ##### Parameters #####
        # Mode
        self.plan = 0 # 0:USV Search , 1:Drn Search
        # Increment of change for USV DP ref point
        self.increment = 1 
        # Maximum DP bias (only for plan 1: Drn Search)
        self.MAX_BIAS_PARAM = 3  
        ##### end Parameters #####
        self.stage = 0
        self.mp_msg = Float32MultiArray()
        self.dp_ref = Float32MultiArray()
        self.max_bias = -1
        self.curr_bias = 0
        self.prev_bias = 0
        self.search_dir = -1 # downward if -1, upward if 1
        
        # Subscribers
        rospy.Subscriber('/intervention_stage', Float32MultiArray, self.update_stage)
        rospy.Subscriber('/vessel_length', Float32, self.update_vessel_len)
        rospy.Subscriber('/dp_ref', Float32MultiArray, self.update_dp_status)
        # Publishers
        self.pub_mp_stage = rospy.Publisher('/intervention_stage', Float32MultiArray, queue_size=10)
        self.pub_dp_ref = rospy.Publisher('/dp_ref', Float32MultiArray, queue_size=10)
        # TODO streaming publisher self.pub_streaming = rospy.Publisher('/', , queue_size=10)
    
    def reinit_search_param(self):
        rospy.loginfo("reinit_search_param")
        self.curr_bias = 0
        self.prev_bias = 0
        self.search_dir = -1 # downward if -1, upward if 1

    def update_vessel_len(self,data):
        rospy.loginfo("update_vessel_len")
        vessel_len = data.data
        max_bias_ = vessel_len*0.3 # consider margin
        if self.plan == 0: # for plan 0: USV Search
            self.max_bias = max_bias_
                
        elif self.plan == 1: # for plan 1: Drn Search
            if max_bias_ < self.MAX_BIAS_PARAM:
                self.max_bias = max_bias_
            else:
                self.max_bias = self.MAX_BIAS_PARAM
    
    def update_stage(self,data):
        rospy.loginfo("update_stage %d",self.stage)
        # Stage 0: Streaming Transition & DP(5m) & Drn Deployment
        if self.stage == 0:
            # Transition to Intervention
            # Debug
            rospy.loginfo("before")
            print(data.data)
            print(len(data.data))
            print(data.data == [0.0, 0.0])
            rospy.loginfo("after")
            # Debug end
            if data.data[0] == 0.0 and data.data[1] == 0.0:
                # Start DP origin
                self.dp_ref.data = [1, 0] 
                self.pub_dp_ref.publish(self.dp_ref)

        # Stage 1: Small Obj Search
        elif self.stage == 1:
            # Start DP
            if data.data[0] == 1.0 and data.data[1] == 0.0:
                # Start DP from origin
                self.dp_ref.data = [1, 0] 
                self.pub_dp_ref.publish(self.dp_ref)

            elif data.data[0] == 1.0 and data.data[1] == 2.0:
                self.update_dp_ref() 

            elif data.data[0] == 2.0 and data.data[1] == 0.0:
                self.reinit_search_param()
                # Update Stage
                self.stage = 2 

                # Stop
                self.dp_ref.data = [0, 0] 
                self.pub_dp_ref.publish(self.dp_ref)

        # Stage 2: Small Obj Retrieval
        elif self.stage == 2:
            self.stage = 3 # if receive [3 0]
        
        # Stage 3: Large Obj Confirmation
        elif self.stage == 3:
            # Start DP
            if data.data[0] == 3.0 and data.data[1] == 0.0:
                # Start DP from origin
                self.dp_ref.data = [1, 0] 
                self.pub_dp_ref.publish(self.dp_ref)

            elif data.data[0] == 3.0 and data.data[1] == 2.0:
                self.update_dp_ref() 

    def update_dp_status(self,data):
        rospy.loginfo("update_dp_status %d",self.stage)
        # Stage 0: Streaming Transition & DP(5m) & Drn Deployment
        if self.stage == 0:
            # DP convergence
            if len(data.data) == 1 and data.data[0] == 1:
                # Stop thrusters 
                self.dp_ref.data = [0, 0] 
                self.pub_dp_ref.publish(self.dp_ref)

            # Thrusters stopped for lift-off
            elif len(data.data) == 1 and data.data[0] == 0:
                # Update Stage
                self.stage = 1

                # Stop streaming
                # TODO self.pub_streaming.publish( )

                # Publish Drn Streaming / Lift-off msg 
                self.mp_msg.data = [0,1]
                self.pub_mp_stage.publish(self.mp_msg)

        # Stage 1: Small Obj Search
        elif self.stage == 1:
            if len(data.data) == 1 and data.data[0] == 1:
                # Publish approach target / start search
                self.mp_msg.data = [1,1]
                self.pub_mp_stage.publish(self.mp_msg)
            
        # Stage 2: Small Obj Retrieval
        elif self.stage == 2:
            if len(data.data) == 1 and data.data[0] == 0:
                # Update Stage
                self.stage = 3

                # Publish USV Stopped msg for small obj release 
                self.mp_msg.data = [2,1]
                self.pub_mp_stage.publish(self.mp_msg)
        
        # Stage 3: Large Obj Confirmation
        elif self.stage == 3:
            if len(data.data) == 1 and data.data[0] == 1:
                # Publish approach target / start search
                self.mp_msg.data = [3,1]
                self.pub_mp_stage.publish(self.mp_msg)

    def update_dp_ref(self):
        rospy.loginfo("update_dp_ref %f",self.curr_bias)
        assert self.max_bias>0, "No target vessel data received"

        # Downward search
        if self.search_dir == -1:
            self.curr_bias = self.curr_bias - self.increment
            if math.fabs(self.curr_bias) > self.max_bias:
                self.curr_bias = -self.max_bias

                # Reached the end. Search the other direction
                self.search_dir = 1
        
        # Upward search
        elif self.search_dir == 1:
            self.curr_bias = self.curr_bias + self.increment
            if math.fabs(self.curr_bias) > self.max_bias:
                self.curr_bias = self.max_bias

                # Reached the end. Search the other direction
                self.search_dir = -1

        self.dp_ref.data = [1, self.curr_bias] 
        self.pub_dp_ref.publish(self.dp_ref)

    def main(self):
        rospy.loginfo("sanity check")
        rospy.spin()

if __name__ == '__main__':
    try:
        mp = IntervetionMP()
        mp.main()
    except rospy.ROSInterruptException:
        pass