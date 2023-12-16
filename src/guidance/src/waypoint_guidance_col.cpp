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
#include "std_msgs/Float64MultiArray.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "core_msgs/string_w_header.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"


using namespace std::chrono_literals;

using std::placeholders::_1;

class GuidanceController
{
  public:
    GuidanceController()
    : nh_("nh_"), count_(0)
    {
      Imu_pose_sub = nh_.subscribe("/imu/data", 10, &GuidanceController::Imu_pose_sub_callback, this);
      sub_utm = nh_.subscribe("/utm_pos", 1000, &GuidanceController::utm_callback, this);
	  sub_radar = nh_.subscribe("/radar_state",1000,&GuidanceController::radar_callback, this);
	  //sub_lidar = nh_.subscribe("/lidar_state",1000,&GuidanceController::lidar_callback, this);
	  sub_wp = nh_.subscribe("/wp",1000,&GuidanceController::waypoint_callback, this);
	  sub_stage = nh_.subscribe("/usv_flag",1000,&GuidanceController::stage_callback, this);
	  sub_K_p = nh_.subscribe("/gain",1000,&GuidanceController::gain_callback, this);
	  //sub_target = nh_.subscribe("/rel_state",1000, &GuidanceController::target_callback, this);


      pub_thrust_left = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/left", 10);
      pub_thrust_right = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/right", 10);
	  pub_reach = nh_.advertise<std_msgs::Int32>("/Flag_4", 10);
      timer_ = nh_.createTimer(ros::Duration(0.1), &GuidanceController::timer_callback, this);
      //nh_.param<int>("V", velocity, 1000);    //기본 threshold thruster force 1000
      //nh_.param<float>("r", radius, 30);
      //nh_.param<float>("K", K, 1300);
      //nh_.param<float>("Kd", Kd, 400);
      //nh_.param<float>("Ki", Ki, 0);

      velocity = 1000;
      radius = 30;
      pose_x = 0;
      pose_y = 0;
      prev_dtheta = 0;
      i = 0;
      K = 3000;
      Kd = 0;   //400
      Ki = 0;     // 0
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

      while (yaw<-M_PI){
          yaw = yaw + 2*M_PI;
      }
	  while (yaw>M_PI){
		  yaw - yaw - 2*M_PI;
	  }

    }


	void waypoint_callback(const std_msgs::Float64MultiArray::ConstPtr msg)    //  wp와 obstacle 동시에 들어옴. 추후 filtering 필요
	{
		ROS_INFO("Waypoint callback");
		wp_call = 1;   //  waypoint가 들어오면 timer 돌게, 들어오면 radar_callback에서 비교 가능
		std::vector <double> list;
		list.clear();
		list = msg->data;
		lla2utm_target(list[0], list[1], 0);
		wp_x_temp = utmt_[0];
		wp_y_temp = utmt_[1];
		utmt_.clear();
		

	}

	void utm_callback(const geometry_msgs::PointStamped::ConstPtr msg)
	{
		pose_x = msg->point.x;
		pose_y = msg->point.y;
	}

	void gain_callback(const std_msgs::Float64::ConstPtr msg)
	{
		K = msg->data;
	}


	void radar_callback(const std_msgs::Float64MultiArray::ConstPtr msg)    //  wp와 obstacle 동시에 들어옴. 추후 filtering 필요 장애물 선박 중에 제일 가까운 선박에 대한 x,y,v_x,v_y 결정, //obstacle make [obs_x,obs_y,obs_v_x,obs_v_y]
	{
		
		ROS_INFO("Radar callback");
		std::vector <double> list; // list는 radar call back에서 오는 전체 list를 받기 위해
		// std::vector <double> obs_x;
		// std::vector <double> obs_y;
		// std::vector <double> obs_v_x;
		// std::vector <double> obs_v_y;
		list.clear();
		// obs_x.clear();
		// obs_y.clear();
		// obs_v_x.clear();
		// obs_v_y.clear();
		list = msg -> data;
		lla2utm_obs(list[0], list[1], 0);
		near_x_t = utmo_[0];
		near_y_t = utmo_[1];
		near_v_x_t = 0;
		near_v_y_t = 0;
		utmo_.clear();
		
		// pose_x = list[0]; //own ship x
		// pose_y = list[1]; //own ship y 
		// int N = list[2];
		// if(wp_call)  // wp 가 들어왔을 때, wp_x_temp, wp_y_temp와, 모든 target들에 대해 비교하고, obstacle_list만 따로 추가하기.
		// {
		// 	//obs_x, obs_y , eliminate 
		// 	for( int i = 0;  i < N; i++)
		// 	{
		// 		bool temp;
				
		// 		if(sqrt((list[4*i+3]-wp_x_temp)*(list[4*i+3]-wp_x_temp)+ (list[4*i+4]-wp_y_temp)*(list[4*i+4]-wp_y_temp)) > wp_obs_thres)
		// 		{
		// 			temp = 1;
		// 		}
		// 		else
		// 		{
		// 			temp = 0;
		// 		}
				

		// 		if(temp)
		// 		{
		// 			obs_x.push_back(list[4*i+3]);
		// 			obs_y.push_back(list[4*i+4]);
		// 			obs_v_x.push_back(list[4*i+5]);
		// 			obs_v_y.push_back(list[4*i+6]);
		// 		}

		// 	}
		// 	// find nearest obstacle target
		// 	std::vector <double> dist_list;
		// 	dist_list.clear();
		// 	for( int k = 0;  k < obs_x.size(); k++)
		// 	{
		// 		double x_t = obs_x[k]-pose_x;
		// 		double y_t = obs_y[k]-pose_y;
		// 		dist_list.push_back(sqrt(x_t*x_t + y_t*y_t));
		// 	}
		// 	if (dist_list.size()>0)
		// 	{
		// 		int min_index = min_element(dist_list.begin(), dist_list.end()) - dist_list.begin();
		// 		near_x_t = obs_x[min_index];
		// 		near_y_t = obs_y[min_index];
		// 		near_v_x_t = obs_v_x[min_index];
		// 		near_v_y_t = obs_v_y[min_index];
		// 	}


		// }
 
	}

	void stage_callback(const std_msgs::Int32::ConstPtr msg)
	{
		stage = msg->data;
	}

	void Cal_rel_state()
	{
		double v_x_o = V_norm*cos(yaw);
		double v_y_o = V_norm*sin(yaw);
		near_v_x_r = v_x_o-near_v_x_t;
		near_v_y_r = v_y_o-near_v_y_t;
		near_x_r = near_x_t-pose_x;
		near_y_r = near_y_t-pose_y;
	}


	// void lidar_callback(const std_msgs::Float64MultiArray::ConstPtr msg)
	// {
		
	// 	std::vector <double> list;
	// 	list = msg->data;
	// 	if(list.size() == 4)
	// 	{
	// 		ROS_INFO("Valid target callblack");
	// 		near_x_r = list[0];
	// 		near_y_r = list[1];
	// 		near_v_x_r = list[2];
	// 		near_v_y_r = list[2];
	// 	}

	// }

	void Cal_App() 
	{
		Dist = sqrt(near_x_r*near_x_r + near_y_r*near_y_r);
		TCPA = -(-near_v_x_r*near_x_r + -near_v_y_r*near_y_r)/(near_v_x_r*near_v_x_r + near_v_y_r*near_v_y_r);
		//DCPA = sqrt((near_x_r + near_v_x_r*TCPA)*(near_x_r + near_v_x_r*TCPA) + (near_y_r + near_v_y_r*TCPA)*(near_y_r + near_v_y_r*TCPA))
		if(Dist< dist_thres)  // only using dist ..
		{
			bApp = 1;
		}
		else
		{
			bApp = 0;
		}
		if(TCPA <0)
		{
			bApp = 0;
		}

	}


	void DM()
	{
		Cal_rel_state(); // x_r, y_r,v_x_r,v_y_r cal
		Cal_App(); //  x_r, y_r, cal distance, cal app
	    
		if(bApp){                           // Approach
			ROS_INFO("Dist: %f", Dist);
			ROS_INFO("R_s: %f", R_s);
			double theta_VO;
			if(Dist>R_s)
			{
				theta_VO = asin(R_s/Dist);
				ROS_INFO("theta_VO: %f", theta_VO);
			}
			else{
				theta_VO = M_PI/2;
				ROS_INFO("theta_VO: %f", theta_VO);
			}
			ROS_INFO("theta_VO: %f", theta_VO);
			double theta_bearing = atan2(near_y_r,near_x_r);
			ROS_INFO("theta_bearing: %f", theta_bearing);
			double theta_v_r = atan2(near_v_y_r,near_v_x_r);
			double theta_out_center = atan2(-near_y_r,-near_x_r);
			std::vector<double> psi_candi = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70};
			std::vector<double> cost_wp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    // wp cost initialization 
			std::vector<double> cost_VO = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    // VO cost initialization
			// cost construction for delta psi
			for(int i = 0;  i < psi_candi.size(); i++)
			{
				double Psi_ = yaw + M_PI/180.0*psi_candi[i];
				while(Psi_>M_PI)
				{
					Psi_ = Psi_ - 2*M_PI;
				}
				while(Psi_<-M_PI)
				{
					Psi_ = Psi_ + 2*M_PI;
				}
				cost_wp[i] = fabs(Psi_-psi_wp); // wp cost construction for ith del psi
				double v_x_r_i = V_norm*cos(Psi_)-near_v_x_t; // relative velocity for i th del_psi
				double v_y_r_i = V_norm*sin(Psi_)-near_v_y_t; 
				double theta_v_r_i = atan2(v_y_r_i,v_x_r_i);
				if(fabs(theta_v_r_i-theta_bearing)<theta_VO){    // in VO
					cost_VO[i] = 1.0;
				}
				else{
					cost_VO[i] = 0.0;  // out of VO
				}
			}
			// find minimum cost and psi_d
			double dcost = 10000.0; // initialize large value
			for(int i = 0; i < psi_candi.size(); i++)
			{
				double Psi_ = yaw + M_PI/180.0*psi_candi[i]; // yaw
				while(Psi_>M_PI)
				{
					Psi_ = Psi_ - 2*M_PI;
				}
				while(Psi_<-M_PI)
				{
					Psi_ = Psi_ + 2*M_PI;
				}
				double cost = cost_wp[i] + 5*cost_VO[i];   // Total cost 
				if(cost < dcost){     // 현재 cost가 그 전의 optimal dcost 보다 작을때 계속 업데이트
					psi_d = Psi_;
					dcost = cost;
				}
			}
			if (Dist<=0.4*R_s)
			{
				if(M_PI/180*fabs(yaw-theta_out_center)>10)
				{
					velocity = 0;
					psi_d = theta_out_center;
				}
				else 
				{
					velocity = 500;
					psi_d = yaw;
					//std::vector<double> theta_out = {2*cos(theta_out_center),2*sin(theta_out_center)};
					//std::vector<double> theta_wp = {1*cos(psi_wp), 1*sin(psi_wp)};
					//psi_d = atan2(theta_wp[1]+theta_out[1], theta_wp[2]+theta_out[2]); 
				}
			}		
			else if (Dist <0.6*R_s && Dist > 0.4*R_s)
			{
				std::vector<double> theta_out = {2*cos(theta_out_center),2*sin(theta_out_center)};
				std::vector<double> theta_wp = {1*cos(psi_wp), 1*sin(psi_wp)};
				psi_d = atan2(theta_wp[0]+theta_out[0], theta_wp[0]+theta_out[0]); 
			}				

		}
		else{ // No app, psi_d is psi_wp waypoint course
			psi_d = psi_wp;
		}
	}


    //void gps_callback(const core_msgs::string_w_header::ConstPtr &msgs)
    //{
      //int zone_ = 52;
      //std::string str_gps = "";
      //str_gps = msgs->data;
 

      // msgs->string_location;
      // std::cout<<str_gps;
      //int strg = 0;
      //int str_count = 0;
      //i/nt str_count_prev = 0;
      //int strg_long_s = 0;
      //int strg_long_e = 0;
      //int strg_lat_s = 0;
      //int strg_lat_e = 0;
      //while(str_count < 6)
      //{
        //if(str_gps[strg] == ',')
        //{
          //str_count = str_count + 1;
       //}

        //if(str_count == 2 && str_count_prev == 1)
        //{
          //strg_long_s = strg;
        //}
        //else if(str_count == 3&& str_count_prev==2)
        //{
          //strg_long_e = strg;
       // }
       // else if(str_count == 4&& str_count_prev==3)
        //{
          //strg_lat_s = strg;
        //}
        //else if(str_count == 5&& str_count_prev==4)
        //{
          //strg_lat_e = strg;
       //}          

        //strg =strg + 1;
        //str_count_prev = str_count;

      //}

      //std::string latitude_str = str_gps.substr(strg_long_s+1,strg_long_e-strg_long_s-1);
      //std::string longitude_str = str_gps.substr(strg_lat_s+1,strg_lat_e-strg_lat_s-1);



      //double latitude_time = std::stod(latitude_str);
      //double longitude_time = std::stod(longitude_str);
      // ROS_INFO("position latitude : %f, longitude : %f ", latitude_time, longitude_time);


      //double latitude = floor(latitude_time*0.01) + (latitude_time - floor(latitude_time*0.01)*100)/60;
      //double longitude = floor(longitude_time*0.01) + (longitude_time - floor(longitude_time*0.01)*100)/60;

      // ROS_INFO_STREAM(str_gps);
      // ROS_INFO_STREAM(latitude);
      // ROS_INFO_STREAM(longitude);
      // Naive Filtering
      // ROS_INFO("position latitude : %f, longitude : %f ", latitude, longitude);
      //if (latitude == 0.0 && longitude == 0.0)
      //{
         // ROS_INFO("The GPS data is not valid | Case 0: Zero Values from GPS");
          //utm_.clear();
      //}
      //else if (longitude > ((zone_ * 6) - 180) || longitude < (((zone_ * 6) - 180) - 6))
      //{ // Check UTM Zone 52
         // ROS_INFO("The GPS data is not valid | Case 1: Measurement out of UTM Zone");
          //utm_.clear();
      //}
      //else    // It has no error
      //{
          //filter code here
          /////////
          //filter code here
          //lla2utm(latitude, longitude, 0);

          //own_ship_info.pose.pose.position.x = utm_[0];
         // own_ship_info.pose.pose.position.y = utm_[1];
          //pose_x = utm_[0];
          //pose_y = utm_[1];

          //utm_.clear();
      //}
    //}


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

	void lla2utm_obs(double latitude, double longitude, double height)
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




        utmo_.push_back(east);
        utmo_.push_back(north);
        // utm_.push_back(height);
    }



    void timer_callback(const ros::TimerEvent& event){

      nh_.getParam("/V", velocity);
      nh_.getParam("/r", radius);
      std_msgs::Int16 thrust_left_msg;
      std_msgs::Int16 thrust_right_msg;      
	  //double lat_0 = 36.919106;
      //double lon_0 = 126.919688;    //126.919688
	 // lla2utm(lat_0, lon_0, 0);
	  //pose_x = utm_[0];
	  //pose_y = utm_[1];
	  //utm_.clear();

      // ROS_INFO( "Velocity : %f", velocity);
      // ROS_INFO( "radius : %f", radius);
      
      //Way Point
      //float WPT_gps[] = {36.599024, 126.300975, 36.597766, 126.299323, 36.597654, 126.301383, 36.597654, 126.303700, 36.596198, 126.301855};
      //float WPT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      //for (int ii = 0;ii<sizeof(WPT_gps) / sizeof(WPT_gps[0])*0.5;ii++){
        //lla2utm_target(WPT_gps[2*ii],WPT_gps[2*ii+1],10);
        //WPT[2*ii] = utmt_[0];
        //WPT[2*ii+1] = utmt_[1];
        //utmt_.clear();
      //}

	  if(wp_call)
	  {
		  
		  if (stage == 3)
		  {
			  //wp_x.assign(wp_x_temp.begin(),wp_x_temp.end());
			  //wp_y.assign(wp_y_temp.begin(),wp_y_temp.end());
			  wp_x = wp_x_temp;
			  wp_y = wp_y_temp;
			  // Pursuit Guidance control Gain
			  // auto K = 1300;
			  // auto Kd = 400;
			  // auto Ki = 0;
			  dt = 0.1;
			  //1500
			  //40
			  //2



			  float P = 30; // waypoint acceptance radius





			  //auto theta = atan2((WPT[2*i + 1]-pose_y),(WPT[2*i ]-pose_x) );   //-pi to pi
			  auto theta = atan2((wp_y-pose_y),(wp_x-pose_x) );   //-pi to pi
			  //if ((-pose_x+WPT[2*i]) > 0 && (-pose_y+WPT[2*i ]) > 0){
				//theta = atan((WPT[2*i +1]-pose_y)/(WPT[2*i +0]-pose_x) );
			  //}
			  //else if ((-pose_x+WPT[2*i ]) < 0 && (-pose_y+WPT[2*i +1]) > 0){
				//theta = 3.141592 + atan((WPT[2*i +1]-pose_y)/(WPT[2*i ]-pose_x) );
			  //}
			  //else if ((-pose_x+WPT[2*i ]) < 0 && (-pose_y+WPT[2*i +1]) < 0){
				//theta = 3.141592 + atan((WPT[2*i +1]-pose_y)/(WPT[2*i +0]-pose_x) );
			  //}      
			  //else{
				//theta = 2*3.141592 + atan((WPT[2*i + 1]-pose_y)/(WPT[2*i ]-pose_x) );
			  //}
			  // psi_wp 0~2*pi
			  while (theta<-M_PI){
				  theta = theta + 2*M_PI;
			  }
			  while (theta>M_PI){
				  theta = theta -2*M_PI;
			  }

			  psi_wp = theta;

			  DM();    //psi_d 결정

			  auto dtheta = psi_d - yaw;



			  while (dtheta > M_PI){
				dtheta -= 2*M_PI;
			  }
			  while (dtheta < -M_PI){
				dtheta += 2*M_PI;
			  }

      

			  if (sqrt(((pose_x - wp_x)*(pose_x - wp_x) + (pose_y - wp_y)*(pose_y - wp_y)))< P){
				// Mission planning으로 도달했다고 publish 필요
				ROS_INFO("ARRIVE!!!!!!!!!!!!!!!");
				std_msgs::Int32 flag;
				flag.data = 1;
				pub_reach.publish(flag);
				
			  }


			 

			  error = error + dtheta*dt;


			  int U = round(K*dtheta + Kd*(dtheta-prev_dtheta)/dt + Ki*error); 

			  prev_dtheta = dtheta;



			 
			  thrust_left_msg.data = velocity - U;
			  thrust_right_msg.data = velocity + U;

			  if(fabs(thrust_left_msg.data - prev_thrust_l) >  dThrust)
			  {
			  	if(thrust_left_msg.data -prev_thrust_l>=0)
			  	{ 
				   thrust_left_msg.data = prev_thrust_l + dThrust;
				}
				else
				{
				   thrust_left_msg.data = prev_thrust_l - dThrust;
				}
			  }
			  if(fabs(thrust_right_msg.data - prev_thrust_r) >  dThrust)
			  {
				if(thrust_right_msg.data -prev_thrust_r>=0)
			  	{ 
				   thrust_right_msg.data = prev_thrust_r + dThrust;
				}
				else
				{
				   thrust_right_msg.data = prev_thrust_r - dThrust;
				}
			  }

			  if (thrust_right_msg.data > 1400){
				 thrust_right_msg.data = 1400;
			  }
			  if (thrust_left_msg.data > 1400){
				 thrust_left_msg.data = 1400;
			  }
			  if (thrust_right_msg.data < -800){
				 thrust_right_msg.data = -800;
			  }
			  if (thrust_left_msg.data < -800){
				 thrust_left_msg.data = -800;
			  }

			  prev_thrust_l = thrust_left_msg.data;
			  prev_thrust_r = thrust_right_msg.data;
        





			  ROS_INFO("===================================================");
			  ROS_INFO("U: %i", U);
			  ROS_INFO( "Desired heading : %f", psi_d*180/M_PI);
			  ROS_INFO( "Heading error : %f", dtheta*180/M_PI);
			  ROS_INFO("position x : %f, y : %f ", pose_x, pose_y);  
			  ROS_INFO("Distance error : %f", sqrt(((pose_x - wp_x)*(pose_x - wp_x) + (pose_y - wp_y)*(pose_y - wp_y))));
			  ROS_INFO("Distance between own ship and the nearest target : %f", Dist);
			  ROS_INFO("yaw : %f", yaw*180/M_PI);
			  ROS_INFO("K_p : %f", K); 
			  ROS_INFO( "Publishing thrust input: left -> %i, right -> %i", thrust_left_msg.data, thrust_right_msg.data);
			  ROS_INFO("===================================================");
      
			  if (yaw > 0.001 || yaw < -0.001){
				pub_thrust_left.publish(thrust_left_msg);
				pub_thrust_right.publish(thrust_right_msg);
				// ROS_INFO( "Send: %f", U);
			  }
		  }
		  else{
			    ROS_INFO( "--- Not WP follwing --");
			    thrust_left_msg.data = 0;
				thrust_right_msg.data = 0;
				pub_thrust_left.publish(thrust_left_msg);
				pub_thrust_right.publish(thrust_right_msg);
				prev_thrust_l = thrust_left_msg.data;
			    prev_thrust_r = thrust_right_msg.data;
		  }


	  }
	  else{
		   ROS_INFO( "---Ready---");
	  }


    }


    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber Imu_pose_sub;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_utm;
	ros::Subscriber sub_lidar;
	ros::Subscriber sub_radar;
	ros::Subscriber sub_wp;
	ros::Subscriber sub_stage;
	ros::Subscriber sub_K_p;
    ros::Publisher pub_thrust_left;
    ros::Publisher pub_thrust_right;
	ros::Publisher pub_reach;

    size_t count_;
    double pose_x;    //36.919106
    double pose_y;    //126.919688
    double yaw = -27*M_PI/180;
    int velocity;
    double kp_ct;
    double kd_ct;
    double kp_h;
    double kd_h;
    double center_x;
    double center_y;
    double radius;
    double prev_dtheta;
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
	std::vector<double> utmo_;
    nav_msgs::Odometry own_ship_info;
	double dist_thres = 90;
	double wp_obs_thres = 5;
	double near_x_r = 10000;
	double near_y_r = 10000;
	double near_v_x_r = 0;
	double near_v_y_r = 0;
	double near_x_t = 10000;
	double near_y_t = 10000;
	double near_v_x_t = 0;
	double near_v_y_t = 0;
	double Dist = 10000;
	double DCPA = 10000;
	double TCPA = -1;
	bool bApp = 0;
	bool wp_call = 0;
	double psi_wp;    
	double psi_d;
	double V_norm = 4.0; // Approximately assume
	double R_s = 40;
	double dThrust = 50;  // 0.1s --> 20 rp,
	double prev_thrust_l = 0.0;
	double prev_thrust_r = 0.0;

	double wp_x;    //36.915192, 36.913557
    double wp_y;    //126.929732, 126.920221
	double wp_x_temp;
	double wp_y_temp;
	// obs_x : 36.918872, obs_y : 126.920503

	int stage = 0;



};

// int main(int argc, char * argv[])
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_guidance");
  GuidanceController guidance_controller;
  ros::spin();
  return 0;
}
