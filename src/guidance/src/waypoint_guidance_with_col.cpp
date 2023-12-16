#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "core_msgs/string_w_header.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"


using namespace std::chrono_literals;

using std::placeholders::_1;

class GuidanceController
{
  public:
    GuidanceController()
    : nh_("nh_"), count_(0)
    {
      Imu_pose_sub = nh_.subscribe("/imu/data", 10, &GuidanceController::Imu_pose_sub_callback, this); // imu call back
      // sub_gps = nh_.subscribe("/gps_bypass", 1000, &GuidanceController::gps_callback, this); // gps call back
      sub_lidar = nh_.subscribe("/target/pos", 100, &GuidanceController::lidar_callback, this);

      pub_thrust_left = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/left", 10); // pub left thrust
      pub_thrust_right = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/right", 10); // pub right thrust
      timer_ = nh_.createTimer(ros::Duration(0.1), &GuidanceController::timer_callback, this);
      nh_.param<int>("V", velocity, 1000);    //기본 threshold thruster force 1000
      nh_.param<float>("r", radius, 30);
      nh_.param<float>("K", K, 1300);                //Parameter ���� V,r,K,Kd,Ki
      nh_.param<float>("Kd", Kd, 400);
      nh_.param<float>("Ki", Ki, 0);

      velocity = 1000;
      radius = 30;
      pose_x = -1300;
      pose_y = 250;
      prev_dtheta = 0;
      i = 0;
      K = 500;
      Kd = 400;
      Ki = 0;
      pnanosec = 0;
      dtime = 0.0;
      dt = 0.1;
      


      // auto K = 1300;
      // auto Kd = 400;
      // auto Ki = 0;


    }


  private:
    void Imu_pose_sub_callback(const sensor_msgs::Imu::ConstPtr& msg)
    { 

      float nanosec = 0.0;
      
      tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 mat(quat);
      
      // int kk = 1;
      // ROS_INFO( "nanosec : %f", float(kk));

      int s = (msg -> header.stamp.sec)*100000000;
      float nsecs = float(msg -> header.stamp.nsec);
      float secs = float(s)*0.00000001;
      // secs = static_cast<float>(msg -> header.stamp.sec)*1.0;
      // float secs = msg -> header.stamp.sec;
      // float nsecs = msg -> header.stamp.nsec;      
      // std::cout << secs << ' '<<typeid(float(s)).name() << std::endl;        
      // ROS_INFO( "secs : %f", s*1.0);
    
      nanosec = secs + nsecs * 0.000000001;
      
      // ROS_INFO( "nanosec : %f", nanosec);
      if (pnanosec == 0.0)
      {
          pnanosec = nanosec;
      }
      
      dtime = dtime + nanosec - pnanosec;
      
      pnanosec = nanosec;
      // ROS_INFO( "dtime : %f", dtime);
      
      double r, p, y;
      mat.getRPY(r, p, y);
      yaw = y;
      // ROS_INFO("yaw : %f", yaw);

      while (yaw<0){
        yaw = yaw + 2*3.141592;     // if ���ٴ� while
      }
    }

	void relative_state_calblack(const ....)
	{
		// obtain most nearest vessel 
		x_r =  ...;
		y_r =  ...;
		v_x_r = ...;
		v_y_r = ...;
		
		if (math.sqrt(x_r^2 + y_r^2) < Dist_thres){
			bApp = 1;
		}
		else{
			bApp = 0;
		}
		


	}




    void lidar_callback(const geometry_msgs::PoseArray::ConstPtr &msgs)
    {

      float x = 0.0;
      // x = msgs->poses[0].position.x;
      
      pose_x = msgs->poses[0].position.x;
      pose_y = msgs->poses[0].position.y;

      if (pose_x > -0.000000001 && pose_x < 0.000000001){
        pose_x = 0.000000001;
      }
      if (pose_y > -0.000000001 && pose_y < 0.000000001){
        pose_y = 0.000000001;
      }


      // tf2::Quaternion quat(
      //   msg->orientation.x,
      //   msg->orientation.y,
      //   msg->orientation.z,
      //   msg->orientation.w);
      // tf2::Matrix3x3 mat(quat);
      
      // int kk = 1;
      // ROS_INFO( "nanosec : %f", float(kk));


    }



    void timer_callback(const ros::TimerEvent& event){

      nh_.getParam("/V", velocity);
      nh_.getParam("/r", radius);
      std_msgs::Int16 thrust_left_msg;
      std_msgs::Int16 thrust_right_msg;      



      auto theta = atan((pose_y)/(pose_x) ); // ������    east x, north y, 
      // auto theta = atan((WPT[2*i + 1]-pose_y)/(WPT[2*i ]-pose_x) ); 

      if ((pose_x) > 0 && (pose_y) > 0){
        theta = atan((pose_y)/(pose_x) );
      }
      else if ((pose_x) < 0 && (pose_y) > 0){
        theta = 3.141592 + atan((pose_y)/(pose_x) );
      }
      else if ((pose_x) < 0 && (pose_y) < 0){
        theta = 3.141592 + atan((pose_y)/(pose_x) );
        
      }      
      else{
        theta = 2*3.141592 + atan((pose_y)/(pose_x) );
      }
      auto dtheta = theta;
      // auto dtheta = theta - yaw;



      while (dtheta > M_PI){
        dtheta -= 2*M_PI;
      }

	    while (dtheta < -M_PI){
        dtheta += 2*M_PI;
      }

    

      error = error + dtheta*dt;


      // auto U = K*dtheta + Kd*(dtheta-prev_dtheta)/dt + Ki*error;     // ?? dtheta/dt--> (dtheta-dtheta_prev)/dt
      int U = round(K*dtheta + Kd*(dtheta-prev_dtheta)/dt + Ki*error); 

      prev_dtheta = dtheta;



      if (dtime > 0.1)
      {
        thrust_left_msg.data = velocity - U;
        thrust_right_msg.data = velocity + U;

        if (velocity + U > 1900){
          thrust_right_msg.data = 1900;
        }
        if (velocity - U > 1900){
          thrust_left_msg.data = 1900;
        }
        if (velocity + U < 400){
          thrust_right_msg.data = 400;
        }
        if (velocity - U < 400){
          thrust_left_msg.data = 400;
        }
        



        dtime = 0;
      }


      ROS_INFO("===================================================");
      ROS_INFO("U: %i", U);
      ROS_INFO( "Heading error : %f", dtheta*180/M_PI);
      ROS_INFO("position x : %f, y : %f ", pose_x, pose_y);  
      ROS_INFO("Distance error : %f", sqrt(((pose_x)*(pose_x) + (pose_y)*(pose_y))));
      ROS_INFO("yaw : %f", yaw*180/M_PI);
      ROS_INFO( "Publishing thrust input: left -> %i, right -> %i", thrust_left_msg.data, thrust_right_msg.data);
      ROS_INFO("===================================================");
      
      if (yaw > 0.001 || yaw < -0.001){
        pub_thrust_left.publish(thrust_left_msg);
        pub_thrust_right.publish(thrust_right_msg);
        // ROS_INFO( "Send: %f", U);
      }
        // thrust_right_msg.data = 400;
        // thrust_left_msg.data = 400;
        // pub_thrust_left.publish(thrust_left_msg);
        // pub_thrust_right.publish(thrust_right_msg);
        

    }


    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber Imu_pose_sub;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_lidar;
    ros::Publisher pub_thrust_left;
    ros::Publisher pub_thrust_right;

    size_t count_;
    float pose_x;
    float pose_y;
    float yaw;
    int velocity;
    float kp_ct;
    float kd_ct;
    float kp_h;
    float kd_h;
    float center_x;
    float center_y;
    float radius;
    float prev_dtheta;
    int i;
    float dt;
    float error = 0;
    float K = 0;
    float Kd = 0;
    float Ki = 0;
	double x_r = 100000;
	double y_r = 100000;
	double v_x_r = 0.0;
	double v_y_r = 0.0;
	double Dist_thres = 100.0;

	// float secs = 0;
    // float nsecs = 0;    
    // float nanosec = 0;
    float pnanosec;
    float dtime = 0;
    std::vector<double> utm_;
    std::vector<double> utmt_;
    nav_msgs::Odometry own_ship_info;



};

// int main(int argc, char * argv[])
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_guidance");
  GuidanceController guidance_controller;
  ros::spin();
  return 0;
}