#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <list>
#include <vector>
#include <chrono>

// Created by sonny on 23. 9. 8.

//Flag 0 : all stop
//Flag 1 : coastal_nav & tracking & Lidar detection Until Docking
//Flag 2 : TSP on
//Flag 3 : TSP off & WPT on (I give node where the USV have to go)
//Flag 4 : WPT off // Camera detection & following & ReID on
//Flag 5 : video streaming on 
//Flag 6 : OK start D.P. 
class MissionPlanner
{
public:
    MissionPlanner()
        : Flag(0), video(false), timer_started(false)
    {
        publisher_Flag =nh.advertise<std_msgs::Int32>("usv_flag", 10);
        publisher_video =nh.advertise<std_msgs::Bool>("toggle_switch",10);

        subscriber_Flag_1 = nh.subscribe("flag_1", 10, &MissionPlanner::Flag_1_callback, this);
        subscriber_Flag_2 = nh.subscribe("flag_2", 10, &MissionPlanner::Flag_2_callback, this);
        subscriber_Flag_3 = nh.subscribe("flag_3", 10, &MissionPlanner::Flag_3_callback, this);
        subscriber_Flag_4 = nh.subscribe("flag_4", 10, &MissionPlanner::Flag_4_callback, this);
        subscriber_Flag_5 = nh.subscribe("flag_5", 10, &MissionPlanner::Flag_5_callback, this);
        subscriber_Flag_6 = nh.subscribe("flag_6", 10, &MissionPlanner::Flag_6_callback, this);
    }


    void Flag_1_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
        std_msgs::Int32 msg;
    	msg.data = Flag_msg -> data;
        if (Flag == 0 && Flag_msg -> data == 1){
            Flag = 1;
        }
    } 



    void Flag_2_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
        std_msgs::Int32 msg;
    	msg.data = Flag_msg -> data;
        if (Flag == 1 && Flag_msg -> data == 1){
            Flag = 2;
        }
    }

    void Flag_3_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
        std_msgs::Int32 msg;
    	msg.data = Flag_msg -> data;
        if (Flag == 2 && Flag_msg -> data == 1){
            Flag = 3;
        }
        // else{
        //     Flag = 2;
        // }
    }

    void Flag_4_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
        std_msgs::Int32 msg;
    	msg.data = Flag_msg -> data;
        if (Flag == 3 && Flag_msg -> data == 1){
            Flag = 4;
        }
        // else{
        //     Flag = 2;
        // }
    }

    void Flag_5_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
    	std_msgs::Int32 msg_1;
    	msg_1.data = Flag_msg -> data;
        if (Flag == 4 && msg_1.data == 1){
            Flag = 5;
            video = true;
            start_timer = ros::Time::now();
            timer_started = true;
        }
        else if(Flag == 4 && msg_1.data ==0) {
           Flag = 2;
        }
    }

    void Flag_6_callback(const std_msgs::Int32::ConstPtr& Flag_msg)
    {
        std_msgs::Int32 msg;
    	msg.data = Flag_msg -> data;
        if (Flag == 5 && Flag_msg -> data == 1){
            Flag = 6;
        }
    }

    void publishFlag() {
        if (Flag == 5 && timer_started && (ros::Time::now() - start_timer).toSec() >= 120.0) { // 2분 (120초) 체크
            Flag = 2;
            timer_started = false;
        }
        if (Flag == 5){
            std::cout << "time" <<" "<< (ros::Time::now() - start_timer).toSec() << std::endl;
        }
        std_msgs::Int32 msg;
        msg.data = Flag;
        publisher_Flag.publish(msg);
        std::cout << "Flag is" <<" "<< Flag << std::endl;
    }

    void publishVideo() {
        std_msgs::Bool msg;
        msg.data = video;
        if (Flag == 5 || Flag == 6){
            publisher_video.publish(msg);
            std::cout << "video is" <<" "<< video << std::endl;
        }
        else{
            video =false;
            msg.data = video;
            publisher_video.publish(msg);
            std::cout << "video is" <<" "<< video << std::endl;            
        }
    
    }

private:
    int Flag;
    bool video;
    bool timer_started;
    ros::Time start_timer;
    ros::NodeHandle nh;
    ros::Publisher publisher_Flag;
    ros::Publisher publisher_video;

    ros::Subscriber subscriber_Flag_1;
    ros::Subscriber subscriber_Flag_2;
    ros::Subscriber subscriber_Flag_3;
    ros::Subscriber subscriber_Flag_4;
    ros::Subscriber subscriber_Flag_5;
    ros::Subscriber subscriber_Flag_6;
};

int main (int argc, char **argv)
{
        ros::init(argc, argv, "usv_flag");
        MissionPlanner missionPlanner;  // MissionPlanner 객체 생성
        ros::Rate loop_rate(10);  // 10Hz

        while (ros::ok()) {
            missionPlanner.publishFlag();  // Flag 값 publish
            missionPlanner.publishVideo(); 
            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
};
