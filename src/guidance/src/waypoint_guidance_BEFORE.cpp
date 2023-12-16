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


using namespace std::chrono_literals;

using std::placeholders::_1;

class GuidanceController
{
  public:
    GuidanceController()
    : nh_("nh_"), count_(0)
    {
      Imu_pose_sub = nh_.subscribe("/imu/data", 10, &GuidanceController::Imu_pose_sub_callback, this);
      sub_gps = nh_.subscribe("/gps_bypass", 1000, &GuidanceController::gps_callback, this);


      pub_thrust_left = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/left", 10);
      pub_thrust_right = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/right", 10);
      timer_ = nh_.createTimer(ros::Duration(0.1), &GuidanceController::timer_callback, this);
      nh_.param<int>("V", velocity, 1000);    //기본 threshold thruster force 1000
      nh_.param<float>("r", radius, 30);
      nh_.param<float>("K", K, 1300);
      nh_.param<float>("Kd", Kd, 400);
      nh_.param<float>("Ki", Ki, 0);

      velocity = 1000;
      radius = 30;
      pose_x = -1300;
      pose_y = 250;
      prev_dtheta = 0;
      i = 0;
      K = 1300;
      Kd = 400;
      Ki = 0;
      pnanosec = 0;
      dtime = 0.0;
      


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

      if (yaw<0){
        yaw = yaw + 2*3.141592;
      }
    }



    void gps_callback(const core_msgs::string_w_header::ConstPtr &msgs)
    {
      int zone_ = 52;
      std::string str_gps = "";
      str_gps = msgs->data;
 

      // msgs->string_location;
      // std::cout<<str_gps;
      int strg = 0;
      int str_count = 0;
      int str_count_prev = 0;
      int strg_long_s = 0;
      int strg_long_e = 0;
      int strg_lat_s = 0;
      int strg_lat_e = 0;
      while(str_count < 6)
      {
        if(str_gps[strg] == ',')
        {
          str_count = str_count + 1;
        }

        if(str_count == 2 && str_count_prev == 1)
        {
          strg_long_s = strg;
        }
        else if(str_count == 3&& str_count_prev==2)
        {
          strg_long_e = strg;
        }
        else if(str_count == 4&& str_count_prev==3)
        {
          strg_lat_s = strg;
        }
        else if(str_count == 5&& str_count_prev==4)
        {
          strg_lat_e = strg;
        }          

        strg =strg + 1;
        str_count_prev = str_count;

      }

      std::string latitude_str = str_gps.substr(strg_long_s+1,strg_long_e-strg_long_s-1);
      std::string longitude_str = str_gps.substr(strg_lat_s+1,strg_lat_e-strg_lat_s-1);



      double latitude_time = std::stod(latitude_str);
      double longitude_time = std::stod(longitude_str);
      // ROS_INFO("position latitude : %f, longitude : %f ", latitude_time, longitude_time);


      double latitude = floor(latitude_time*0.01) + (latitude_time - floor(latitude_time*0.01)*100)/60;
      double longitude = floor(longitude_time*0.01) + (longitude_time - floor(longitude_time*0.01)*100)/60;

      // ROS_INFO_STREAM(str_gps);
      // ROS_INFO_STREAM(latitude);
      // ROS_INFO_STREAM(longitude);
      // Naive Filtering
      // ROS_INFO("position latitude : %f, longitude : %f ", latitude, longitude);
      if (latitude == 0.0 && longitude == 0.0)
      {
          ROS_INFO("The GPS data is not valid | Case 0: Zero Values from GPS");
          utm_.clear();
      }
      else if (longitude > ((zone_ * 6) - 180) || longitude < (((zone_ * 6) - 180) - 6))
      { // Check UTM Zone 52
          ROS_INFO("The GPS data is not valid | Case 1: Measurement out of UTM Zone");
          utm_.clear();
      }
      else    // It has no error
      {
          //filter code here
          /////////
          //filter code here
          lla2utm(latitude, longitude, 0);

          own_ship_info.pose.pose.position.x = utm_[0];
          own_ship_info.pose.pose.position.y = utm_[1];
          pose_x = utm_[0];
          pose_y = utm_[1];

          utm_.clear();
      }
    }


    void lla2utm(double latitude, double longitude, double height)
    {


        

        float prev_x = 0.0;
        float prev_y = 0.0;
        float prev_v_x = 0.0;
        float prev_v_y = 0.0;

        const double kNN_ = 0;
        const double kNS_ = 10000000.0;
        const double kE0_ = 500000.0;
        const double kPI_ = 3.14159265359;

        double east, north;

        double dLat = latitude * kPI_/180;
        double dLon = longitude * kPI_/180;

        double lon0_f = floor(longitude/6)*6+3;
        double lon0 = lon0_f*kPI_/180;
        double k0 = 0.9996;

        double FE = 500000;
        double FN = (latitude < 0) ? 10000000 : 0;

        double Wa = 6378137;
        double Weps = 0.006739496742333;
        double We = 0.081819190842965;

        double WN = Wa/sqrt(1-pow(We,2)*pow(sin(dLat),2));
        double WT = pow(tan(dLat),2);
        double WC = (pow(We,2)/(1-pow(We,2)))*pow(cos(dLat),2);
        double WLA = (dLon - lon0)*cos(dLat);

        double WM = (Wa*((1-pow(We,2)/4 - 3*pow(We,4)/64 - 5*pow(We,6)/256)*dLat-(3*pow(We,2)/8 + 3*pow(We,4)/32 + 45*pow(We,6)/1024)*sin(2*dLat)+(15*pow(We,4)/256 + 45*pow(We,6)/1024)*sin(4*dLat) - (35*pow(We,6)/3072)*sin(6*dLat)));


        east = (FE + k0*WN*(WLA + (1-WT+WC)*pow(WLA,3)/6 + (5-18*WT + pow(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120));
        north =(FN + k0*WM + k0*WN*tan(dLat)*(pow(WLA,2)/2 + (5-WT + 9*WC + 4*pow(WC,2))*pow(WLA,4)/24 + (61 - 58*WT + pow(WT,2) + 600*WC - 330*Weps)*pow(WLA,6)/720));




        utm_.push_back(east);
        utm_.push_back(north);
        // utm_.push_back(height);
    }

    void lla2utm_target(double latitude, double longitude, double height)
    {

        float prev_x = 0.0;
        float prev_y = 0.0;
        float prev_v_x = 0.0;
        float prev_v_y = 0.0;

        const double kNN_ = 0;
        const double kNS_ = 10000000.0;
        const double kE0_ = 500000.0;
        const double kPI_ = 3.14159265359;

        double east, north;

        double dLat = latitude * kPI_/180;
        double dLon = longitude * kPI_/180;

        double lon0_f = floor(longitude/6)*6+3;
        double lon0 = lon0_f*kPI_/180;
        double k0 = 0.9996;

        double FE = 500000;
        double FN = (latitude < 0) ? 10000000 : 0;

        double Wa = 6378137;
        double Weps = 0.006739496742333;
        double We = 0.081819190842965;

        double WN = Wa/sqrt(1-pow(We,2)*pow(sin(dLat),2));
        double WT = pow(tan(dLat),2);
        double WC = (pow(We,2)/(1-pow(We,2)))*pow(cos(dLat),2);
        double WLA = (dLon - lon0)*cos(dLat);

        double WM = (Wa*((1-pow(We,2)/4 - 3*pow(We,4)/64 - 5*pow(We,6)/256)*dLat-(3*pow(We,2)/8 + 3*pow(We,4)/32 + 45*pow(We,6)/1024)*sin(2*dLat)+(15*pow(We,4)/256 + 45*pow(We,6)/1024)*sin(4*dLat) - (35*pow(We,6)/3072)*sin(6*dLat)));


        east = (FE + k0*WN*(WLA + (1-WT+WC)*pow(WLA,3)/6 + (5-18*WT + pow(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120));
        north =(FN + k0*WM + k0*WN*tan(dLat)*(pow(WLA,2)/2 + (5-WT + 9*WC + 4*pow(WC,2))*pow(WLA,4)/24 + (61 - 58*WT + pow(WT,2) + 600*WC - 330*Weps)*pow(WLA,6)/720));




        utmt_.push_back(east);
        utmt_.push_back(north);
        // utm_.push_back(height);
    }



    void timer_callback(const ros::TimerEvent& event){

      nh_.getParam("/V", velocity);
      nh_.getParam("/r", radius);
      std_msgs::Int16 thrust_left_msg;
      std_msgs::Int16 thrust_right_msg;      

      // ROS_INFO( "Velocity : %f", velocity);
      // ROS_INFO( "radius : %f", radius);
      
      //Way Point
      float WPT_gps[] = {36.599024, 126.300975, 36.597766, 126.299323, 36.597654, 126.301383, 36.597654, 126.303700, 36.596198, 126.301855};
      float WPT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      for (int ii = 0;ii<sizeof(WPT_gps) / sizeof(WPT_gps[0])*0.5;ii++){
        lla2utm_target(WPT_gps[2*ii],WPT_gps[2*ii+1],10);
        WPT[2*ii] = utmt_[0];
        WPT[2*ii+1] = utmt_[1];
        utmt_.clear();
      }

      // Pursuit Guidance control Gain
      // auto K = 1300;
      // auto Kd = 400;
      // auto Ki = 0;
      dt = 0.1;
      //1500
      //40
      //2



      float P = 15; // waypoint acceptance radius





      auto theta = atan((WPT[2*i + 1]-pose_y)/(WPT[2*i ]-pose_x) );

      if ((-pose_x+WPT[2*i]) > 0 && (-pose_y+WPT[2*i ]) > 0){
        theta = atan((WPT[2*i +1]-pose_y)/(WPT[2*i +0]-pose_x) );
      }
      else if ((-pose_x+WPT[2*i ]) < 0 && (-pose_y+WPT[2*i +1]) > 0){
        theta = 3.141592 + atan((WPT[2*i +1]-pose_y)/(WPT[2*i ]-pose_x) );
      }
      else if ((-pose_x+WPT[2*i ]) < 0 && (-pose_y+WPT[2*i +1]) < 0){
        theta = 3.141592 + atan((WPT[2*i +1]-pose_y)/(WPT[2*i +0]-pose_x) );
        
      }      
      else{
        theta = 2*3.141592 + atan((WPT[2*i + 1]-pose_y)/(WPT[2*i ]-pose_x) );
      }
      auto dtheta = theta - yaw;



      if (dtheta > M_PI){
        dtheta -= 2*M_PI;
      }else if (dtheta < -M_PI){
        dtheta += 2*M_PI;
      }

      

      if (sqrt(((pose_x - WPT[2*i])*(pose_x - WPT[2*i]) + (pose_y - WPT[2*i+1])*(pose_y - WPT[2*i+1])))< P){
        i = i+1;
      }


      float length = sizeof(WPT)/sizeof(*WPT);
      if (i > length/2-1)
      {
        thrust_left_msg.data = 0;
        thrust_right_msg.data = 0;
        pub_thrust_left.publish(thrust_left_msg);
        pub_thrust_right.publish(thrust_right_msg);
        exit(0);
      }


      error = error + dtheta*dt;


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