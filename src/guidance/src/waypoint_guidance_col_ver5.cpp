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
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
//#include "core_msgs/string_w_header.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

// USV control (target vessel following & obstacle avoidance)

using namespace std::chrono_literals;

using std::placeholders::_1;

class GuidanceController
{
  public:
    GuidanceController()
    : nh_("nh_"), count_(0)
    {
      Imu_pose_sub = nh_.subscribe("/imu/data", 10, &GuidanceController::Imu_pose_sub_callback, this);
      //sub_gps = nh_.subscribe("/gps_bypass", 1000, &GuidanceController::gps_callback, this);
	  sub_radar = nh_.subscribe("/global_track",1000,&GuidanceController::radar_callback, this);
	  sub_lidar = nh_.subscribe("/lidar_track",1000,&GuidanceController::lidar_callback, this);
	  sub_wp = nh_.subscribe("/wp_id",1000,&GuidanceController::waypoint_callback, this);
	  sub_stage = nh_.subscribe("/usv_flag",1000,&GuidanceController::stage_callback, this);
	  sub_K_p = nh_.subscribe("/gain",1000,&GuidanceController::gain_callback, this);
	  sub_V = nh_.subscribe("/vel",1000,&GuidanceController::velocity_callback, this);
	  sub_R_s = nh_.subscribe("/radius",1000,&GuidanceController::radius_callback, this);
	  sub_offset = nh_.subscribe("/offset",1000,&GuidanceController::offset_callback, this);
	  //sub_target = nh_.subscribe("/rel_state",1000, &GuidanceController::target_callback, this);


      pub_thrust_left = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/left", 10);
      pub_thrust_right = nh_.advertise<std_msgs::Int16>("/workshop_setup/pods/right", 10);
	  pub_thrust_left_pod = nh_.advertise<std_msgs::Float32>("/workshop_setup/pod_steer/left_steer", 10);
	  pub_thrust_right_pod = nh_.advertise<std_msgs::Float32>("/workshop_setup/pod_steer/right_steer", 10);
	  //pub_arrive = nh_.advertise<std_msgs::Bool>("/reach",10);
	  pub_reach = nh_.advertise<std_msgs::Int32>("/flag_4", 10);
	  pub_reach_7 = nh_.advertise<std_msgs::Int32>("/flag_7", 10);
      timer_ = nh_.createTimer(ros::Duration(0.1), &GuidanceController::timer_callback, this);
      //nh_.param<int>("V", velocity, 1000);    //기본 threshold thruster force 1000
      //nh_.param<float>("r", radius, 30);
      //nh_.param<float>("K", K, 1300);
      //nh_.param<float>("Kd", Kd, 400);
      //nh_.param<float>("Ki", Ki, 0);

      velocity = 600;
	  wp_vel = 600;
      radius = 30;
      pose_x = 0;
      pose_y = 0;
      prev_dtheta = 0;
      i = 0;
      K = 200;
      Kd = 0;   //400
      Ki = 0;     // 0
      pnanosec = 0;
      dtime = 0.0;
      


      // auto K = 1300;
      // auto Kd = 400;
      // auto Ki = 0;


    }


  private:
    void Imu_pose_sub_callback(const sensor_msgs::Imu::ConstPtr& msg) // Subscribe IMU data
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
		  yaw = yaw - 2*M_PI;
	  }

    }



	void waypoint_callback(const std_msgs::Float32::ConstPtr msg)    // Subscribe Target vessel ID
	{
		ROS_INFO("Waypoint callback");
		wp_call = 1;   //  waypoint가 들어오면 timer 돌게, 들어오면 radar_callback에서 비교 가능
		id = int(msg->data);
		//std::vector <float> list;
		//list.clear();
		//list = msg->data;
		//wp_x_temp = list[0];
		//wp_y_temp = list[1];
	}

	void gain_callback(const std_msgs::Float64::ConstPtr msg) // Subscribe Control P gain 
	{
		K = msg->data;
	}

	void velocity_callback(const std_msgs::Int16::ConstPtr msg) // Subscribe normal velocity 
	{
		wp_vel = int(msg->data);
	}
	
	void radius_callback(const std_msgs::Float64::ConstPtr msg) // Subscribe collision avoidance radius 
	{
		R_s = msg->data;
	}

	void offset_callback(const std_msgs::Float32::ConstPtr msg) // Subscribe pod offset 
	{
		offset = msg->data;
	}
	 
	 

	void radar_callback(const std_msgs::Float32MultiArray::ConstPtr msg)    //  Subscirbe target vessel's position or velocity 
	{
		
		ROS_INFO("Radar callback");
		radar_call = 1;
		std::vector <float> list; 
		std::vector <double> target_x;
		std::vector <double> target_y;
		std::vector <double> target_v_x;
		std::vector <double> target_v_y;
		std::vector <int > target_id;
		list.clear();
		target_x.clear();
		target_y.clear();
		target_v_x.clear();
		target_v_y.clear();
		target_id.clear();
		list = msg -> data;
		pose_x = list[0]; //own ship x
		pose_y = list[1]; //own ship y 
		int N = int(list[2]);
		if(wp_call)  
		{
			//_x, obs_y , eliminate 
			for( int i = 0;  i < N; i++)
			{			
				target_id.push_back(int(list[5*i+3]));
				target_x.push_back(list[5*i+4]);
				target_y.push_back(list[5*i+5]);
				target_v_x.push_back(list[5*i+6]);
				target_v_y.push_back(list[5*i+7]);					
			}
			//find wp (correct id)
			int target_index = 0;
			//wp_x_temp = 1000;
			//wp_y_temp = 1000; // initialize
			std::cout << "ID RECV : " << id << std::endl;
			bool find_index = 0;
			for( int k = 0; k <N; k++)
			{
				std::cout << "msg ID : " << target_id[k] << std::endl;
				if(target_id[k] == id)
				{
					target_index = k;
					std::cout <<"Passed!" << std::endl;
					std::cout << target_y[target_index] << std::endl;
					find_index = 1;
				}
			}
			if(target_x.size()>0 && find_index)
			{
				wp_x_temp = target_x[target_index];
				wp_y_temp = target_y[target_index];
				wp_ok = 1;

			}
	
			// wp_x_temp = target_x[target_index];
			// wp_y_temp = target_y[target_index];
			// wp_ok = 1;

			//// find nearest obstacle target
			//std::vector <double> dist_list;
			//dist_list.clear();
			//for( int k = 0;  k < obs_x.size(); k++)
			//{
				//double x_t = obs_x[k]-pose_x;
				//double y_t = obs_y[k]-pose_y;
				//dist_list.push_back(sqrt(x_t*x_t + y_t*y_t));
			//}
			//if (dist_list.size()>0)
			//{
				//int min_index = min_element(dist_list.begin(), dist_list.end()) - dist_list.begin();
				//near_x_t = obs_x[min_index];
				//near_y_t = obs_y[min_index];
				//near_v_x_t = obs_v_x[min_index];
				//near_v_y_t = obs_v_y[min_index];
			//}


		}
 
	}

	void stage_callback(const std_msgs::Int32::ConstPtr msg) // Subscribe current task stage
	{
		stage = msg->data;
	}

	// void Cal_rel_state()
	// {
	// 	double v_x_o = V_norm*cos(yaw);
	// 	double v_y_o = V_norm*sin(yaw);
	// 	near_v_x_r = v_x_o-near_v_x_t;
	// 	near_v_y_r = v_y_o-near_v_y_t;
	// 	near_x_r = near_x_t-pose_x;
	// 	near_y_r = near_y_t-pose_y;
	// }


	void lidar_callback(const std_msgs::Float32MultiArray::ConstPtr msg) // Subscribe lidar detection results
	{
		//ROS_INFO("lidar callback");
		std::vector <double> obs_x_r;
		std::vector <double> obs_y_r;
		std::vector <double> obs_v_x_r;
		std::vector <double> obs_v_y_r;
		std::vector <double> obs_id;
		std::vector <double> obs_yaw;
		std::vector <double> obs_L;
		std::vector <double> obs_D;
		std::vector <float> list;
		list.clear();
		obs_id.clear();
		obs_x_r.clear();
		obs_y_r.clear();
		obs_v_x_r.clear();
		obs_v_y_r.clear();
		obs_yaw.clear();
		obs_L.clear();
		obs_D.clear();
		list = msg->data;
		int N = int(list[0]);
		if(N == 0)
		{
			near_x_r = 10000;
			near_y_r = 10000;
		}
		for (int i = 0; i < N; i++)
		{
			obs_id.push_back(list[8*i+1]);
			obs_x_r.push_back(list[8*i+2]);
			obs_y_r.push_back(list[8*i+3]);
			obs_v_x_r.push_back(list[8*i+4]);
			obs_v_y_r.push_back(list[8*i+5]);
			obs_yaw.push_back(list[8*i+6]);
			obs_L.push_back(list[8*i+7]);
			obs_D.push_back(list[8*i+8]);

		}
		// bool find_del_index = 0;
		// int del_index = 0;
		// for( int k = 0; k <N; k++)
		// {
		// 	// std::cout << "msg ID : " << target_id[k] << std::endl;
		// 	if(obs_id[k] == id)
		// 	{
		// 		del_index = k;
		// 		// std::cout <<"Passed!" << std::endl;
		// 		// std::cout << target_y[target_index] << std::endl;
		// 		find_del_index = 1;
		// 	}
		// }
		// if (find_del_index && obs_x_r.size()>0)
		// {
		// 	vector.erase( vector.begin() + 3 );
		// 	obs_id.erase(obs_id.begin() + del_index);
		// 	obs_x_r.erase(obs_x_r.begin() + del_index);
		// 	obs_y_r.erase(obs_y_r.begin() + del_index);
		// 	obs_v_x_r.erase(obs_v_x_r.begin() + del_index);
		// 	obs_v_y_r.erase(obs_v_y_r.begin() + del_index);
		// }
		std::vector <double> dist_list;
		dist_list.clear();
		for( int k = 0;  k < obs_x_r.size(); k++)
		{
		 	double x_r = obs_x_r[k];
		 	double y_r = obs_y_r[k];
		 	dist_list.push_back(sqrt(x_r*x_r + y_r*y_r));
		}

		if (dist_list.size()>0)
		{
			int min_index = min_element(dist_list.begin(), dist_list.end()) - dist_list.begin();
			near_x_r = obs_x_r[min_index]; //
			near_y_r = obs_y_r[min_index]; //
			near_v_x_r = obs_v_x_r[min_index];  //
			near_v_y_r = obs_v_y_r[min_index];  //
			near_id = int(obs_id[min_index]);
		}
		

	}

	void Cal_App() // Determine whether obstacle avoidance  is necessary
	{
		ROS_INFO("near_x_r: %f", near_x_r);
		ROS_INFO("near_y_r: %f", near_y_r);
		Dist = sqrt(near_x_r*near_x_r + near_y_r*near_y_r);
		TCPA = -(near_v_x_r*near_x_r + near_v_y_r*near_y_r)/(near_v_x_r*near_v_x_r + near_v_y_r*near_v_y_r);
		//DCPA = sqrt((near_x_r + near_v_x_r*TCPA)*(near_x_r + near_v_x_r*TCPA) + (near_y_r + near_v_y_r*TCPA)*(near_y_r + near_v_y_r*TCPA))
		if(id != near_id) // Determine nearest obstacle is wp?
		{
			if(Dist< dist_thres)  // only using dist ..
			{
				if(Dist  < R_s)
				{
					bApp = 1;
				}
				else
				{
					if(TCPA <0)
					{
						bApp = 0;
					}
					else
					{
						bApp = 1;
					}
				}
			}
			else
			{
				
				bApp = 0;
				
			}
			// if(TCPA <0)
			// {
			// 	bApp = 0;
			// }
		}
		else
		{
			bApp = 0;
		}
	

	}

	void Cal_target_vel() // Calculate target's velocity
	{
		near_v_x_t = near_v_x_r + V_norm*cos(yaw);
		near_v_y_t = near_v_y_r + V_norm*sin(yaw);
	}


	void DM() // Determine own ship's target course using cost function
	{
		// Cal_rel_state(); // x_r, y_r,v_x_r,v_y_r cal
		Only_turn = 0;
		Cal_App(); //  x_r, y_r, cal distance, cal app
		Cal_target_vel();
		K = 200;
	    
		if(bApp){                           // Approach
			// K = 200;
			ROS_INFO("Dist: %f", Dist);
			ROS_INFO("R_s: %f", R_s);
			double theta_VO;
			if(Dist>R_s)
			{
				theta_VO = asin(R_s/Dist);
				ROS_INFO("theta_VO: %f", theta_VO*180/M_PI);
			}
			else{
				theta_VO = M_PI/2;
				ROS_INFO("theta_VO: %f", theta_VO);
			}
			
			// ROS_INFO("theta_VO: %f", theta_VO);
			double theta_bearing = atan2(near_y_r,near_x_r);
			double theta_out_center = atan2(-near_y_r,-near_x_r);
			ROS_INFO("theta_bearing: %f", theta_bearing*180/M_PI);
			ROS_INFO("theta_out_center: %f", theta_out_center*180/M_PI);
			// double theta_v_r = atan2(near_v_y_r,near_v_x_r);
			//std::vector<double> psi_candi = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86.87,88,89,90};    // del psi candidate
			std::vector<double> psi_candi = {-70,-69,-68,-67,-66,-65,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70}; 
			//ROS_INFO("1111111111");
			//std::vector<double> cost_wp(71,0);    // wp cost initialization 
			// std::vector<double> cost_wp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			std::vector<double> cost_wp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			//ROS_INFO("2222222222");
			//std::vector<double> cost_VO(71,0);  // VO cost initialization
			// std::vector<double> cost_VO = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			std::vector<double> cost_VO = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			//ROS_INFO("333333333");
			// std::vector<double> cost_port(71,0);
			// std::vector<double> cost_port = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			std::vector<double> cost_vio = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			//ROS_INFO("444444444");
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
				// ROS_INFO("Psi_ : %f", Psi_);
				// ROS_INFO("v_x_r_i: %f", v_x_r_i);
				// ROS_INFO("v_y_r_i: %f", v_y_r_i);
				//ROS_INFO("near_v_x_t: %f",near_v_x_t);
				//ROS_INFO("near_v_y_t: %f",near_v_y_t);
				// ROS_INFO("near_v_x_r: %f",near_v_x_r);
				// ROS_INFO("near_v_y_r: %f",near_v_y_r);
				// ROS_INFO("V_o_x : %f",V_norm*cos(yaw));
				// ROS_INFO("V_o_y : %f",V_norm*sin(yaw));
				double theta_v_r_i = atan2(v_y_r_i,v_x_r_i);
				if(fabs(theta_v_r_i-theta_bearing)<theta_VO){    // in VO
					cost_VO[i] = 1.0;
					//ROS_INFO("Dangerous angle : %f", Psi_*180/M_PI);
					//ROS_INFO("Relative angle: %f", theta_v_r_i*180/M_PI);
				}
				else{
					cost_VO[i] = 0.0;  // out of VO
				}

				if(theta_v_r_i <= theta_bearing)
				{
					cost_vio[i] = 1.0;
				}
				else
				{
					cost_vio[i] = 0.0;
				}
				//if(psi_candi[i]>0)
				//{
					//cost_port[i] = 1.0;
				//}
				
			}
			//ROS_INFO("555555555555");
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
				//double cost = cost_wp[i] + 5*cost_VO[i] + 2*cost_port[i];   // Total cost 
				double cost = cost_wp[i] + 5*cost_VO[i]; // candidate delpsi<0
				if(cost < dcost){     // 
					psi_d = Psi_;
					dcost = cost;
				}
			}	
			// velocity = 600;
			//ROS_INFO("66666666666  psi_d = %f",psi_d);
			if (Dist<=0.6*R_s)
			{
				if(180/M_PI*fabs(yaw-theta_out_center)>10)
				{
					Only_turn = 1;
					//velocity = 0;
					K  = 1300;
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
			else if (Dist <0.85*R_s && Dist > 0.6*R_s)
			{
				velocity = 500;
				std::vector<double> theta_out = {2*cos(theta_out_center),2*sin(theta_out_center)};
				std::vector<double> theta_wp = {1*cos(psi_wp), 1*sin(psi_wp)};
				psi_d = atan2(theta_wp[1]+theta_out[1], theta_wp[0]+theta_out[0]); 
			}
			else if (Dist<R_s){
				velocity = 550;
				std::vector<double> theta_out = {cos(theta_out_center),sin(theta_out_center)};
				std::vector<double> theta_wp = {2*cos(psi_wp), 2*sin(psi_wp)};
				psi_d = atan2(theta_wp[1]+theta_out[1], theta_wp[0]+theta_out[0]); 
			}

			
			// if (Dist<=0.7*R_s)
			// {
			// 	psi_d = theta_out_center;
			// }		

		}
		else{ // No app, psi_d is psi_wp waypoint course
			// K = 200;
			velocity = wp_vel;
			psi_d = psi_wp;
		}
	}


    

    


    void timer_callback(const ros::TimerEvent& event){  // Main loop for usv control

      nh_.getParam("/V", velocity);
      nh_.getParam("/r", radius);
      std_msgs::Int16 thrust_left_msg;
      std_msgs::Int16 thrust_right_msg;     
	  std_msgs::Float32 thrust_left_pod_msg;
	  std_msgs::Float32 thrust_right_pod_msg; 
	  std_msgs::Int32 reach_msg; 
	  std_msgs::Int32 reach_7_msg;

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

	  if(wp_call && radar_call && wp_ok)
	  {
		  
		  if (stage == 3 || stage == 6)
		  {
			  //wp_x.assign(wp_x_temp.begin(),wp_x_temp.end());
			  //wp_y.assign(wp_y_temp.begin(),wp_y_temp.end());
			  //id_init = 1;
			//   wp_x_bef = wp_x_temp;
			//   wp_y_bef = wp_y_temp;

			//   if(sqrt(((pose_x - wp_x_bef)*(pose_x - wp_x_bef) + (pose_y - wp_y_bef)*(pose_y - wp_y_bef))) < 120 && pin_flag && Time > 10)    //wp_rot_call = 1
			//   {
			// 	  pin_point_x = pose_x;
			// 	  pin_point_y = pose_y;
			// 	  pin_flag = 0;
			//   }

			//   if(sqrt(((pose_x - wp_x_bef)*(pose_x - wp_x_bef) + (pose_y - wp_y_bef)*(pose_y - wp_y_bef))) < 120 && Time > 2)
			//   {
			// 	  wp_x = pin_point_x + cos(M_PI/180*5)*(wp_x_bef-pin_point_x) - sin(M_PI/180*5)*(wp_y_bef-pin_point_y);
			// 	  wp_y = pin_point_y + sin(M_PI/180*5)*(wp_x_bef-pin_point_x) + cos(M_PI/180*5)*(wp_y_bef-pin_point_y);
			//   }
			//   else
			//   {
			// 	  wp_x = wp_x_bef;
			// 	  wp_y = wp_y_bef;
			//   }
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



			  float P = 100; // waypoint acceptance radius





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
				  theta = theta - 2*M_PI;
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

      

			  if (sqrt(((pose_x - wp_x)*(pose_x - wp_x) + (pose_y - wp_y)*(pose_y - wp_y)))< P && Time > 3){
				// Mission planning으로 도달했다고 publish 필요
				if(stage == 3)
				{
					reach_msg.data = 1;
					pub_reach.publish(reach_msg);
				}
				if(stage == 6)
				{
					reach_7_msg.data = 1;
					pub_reach_7.publish(reach_7_msg);
				}
				//reach_msg.data = 1;
				//pub_reach.publish(reach_msg);
			  }


			 

			  error = error + dtheta*dt;
			  Time = Time + dt;


			  //int U = round(K*dtheta + Kd*(dtheta-prev_dtheta)/dt + Ki*error); 

			  float U = K*dtheta + Kd*(dtheta-prev_dtheta)/dt + Ki*error; 

			  prev_dtheta = dtheta;



			  if(Only_turn)
			  {
				thrust_left_msg.data = -round(U);
				thrust_right_msg.data = round(U);
				thrust_left_pod_msg.data = offset;  //-8 offset
				thrust_right_pod_msg.data = offset; // -8 offset
			  }
			  else
			  {
				thrust_left_msg.data = velocity;
				thrust_right_msg.data = velocity;
				thrust_left_pod_msg.data = offset+U;  //-8 offset
				thrust_right_pod_msg.data = offset+U; // -8 offset
			  }

			  if(thrust_left_pod_msg.data>60){
				thrust_left_pod_msg.data = 60;
			  }
			  if(thrust_left_pod_msg.data<-60){
				thrust_left_pod_msg.data = -60;
			  }
			  if(thrust_right_pod_msg.data>60){
				thrust_right_pod_msg.data = 60;
			  }
			  if(thrust_right_pod_msg.data<-60){
				thrust_right_pod_msg.data = -60;
			  }

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
			  ROS_INFO("time elapse for each waypoint : %f", Time);
			  ROS_INFO("U: %f", U);
			  ROS_INFO( "Waypoint heading : %f", psi_wp*180/M_PI);
			  ROS_INFO( "Desired heading : %f", psi_d*180/M_PI);
			  ROS_INFO( "Heading error : %f", dtheta*180/M_PI);
			  ROS_INFO("position x : %f, y : %f ", pose_x, pose_y);  
			  ROS_INFO("Distance error : %f", sqrt(((pose_x - wp_x)*(pose_x - wp_x) + (pose_y - wp_y)*(pose_y - wp_y))));
			  ROS_INFO("Distance between own ship and the nearest target : %f", Dist);
			  ROS_INFO("yaw : %f", yaw*180/M_PI);
			  ROS_INFO("K_p : %f", K); 
			  ROS_INFO("vel : %d", velocity);
			  ROS_INFO("Radius : %f", R_s);
			  ROS_INFO("Offset : %f ", offset);
			  ROS_INFO( "Publishing thrust input: left -> %i, right -> %i", thrust_left_msg.data, thrust_right_msg.data);
			  ROS_INFO( "Publishing thrust pod: left -> %f, right -> %f", thrust_left_pod_msg.data, thrust_right_pod_msg.data);
			  ROS_INFO( "WP_ID : %d", id);
			  ROS_INFO( "Nearest_ID : %d", near_id);
			  ROS_INFO("WP_x : %f, WP_y : %f", wp_x, wp_y);
			  //ROS_INFO("pin_point_x : %f, pin_point_y : %f", pin_point_x, pin_point_y);
			  ROS_INFO("===================================================");
      
			  if (yaw > 0.001 || yaw < -0.001){
				pub_thrust_left.publish(thrust_left_msg);
				pub_thrust_right.publish(thrust_right_msg);
				pub_thrust_left_pod.publish(thrust_left_pod_msg);
				pub_thrust_right_pod.publish(thrust_right_pod_msg);
			  }
		  }
		  else{
			    ROS_INFO( "--- Not WP follwing --");
			    // thrust_left_msg.data = 0;
				// thrust_right_msg.data = 0;
				// pub_thrust_left.publish(thrust_left_msg);
				// pub_thrust_right.publish(thrust_right_msg);
				// prev_thrust_l = thrust_left_msg.data;
			    // prev_thrust_r = thrust_right_msg.data;
				//wp_ok = 0;
				//pin_flag = 1;
				//if(id_init)
				//{
					//id = 1010101010;
					//id_init = 0;
				//}
				Time = 0;
				//pin_point_x = 0;
				//pin_point_y = 0;
			
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
	ros::Subscriber sub_lidar;
	ros::Subscriber sub_radar;
	ros::Subscriber sub_wp;
	ros::Subscriber sub_stage;
	ros::Subscriber sub_K_p;
	ros::Subscriber sub_V;
	ros::Subscriber sub_R_s;
	ros::Subscriber sub_offset;
   
	//ros::Publisher pub_arrive;
	ros::Publisher pub_thrust_left;
	ros::Publisher pub_thrust_right;
	ros::Publisher pub_thrust_left_pod;
	ros::Publisher pub_thrust_right_pod;
	ros::Publisher pub_reach;
	ros::Publisher pub_reach_7;

    size_t count_;
    double pose_x;
    double pose_y;
    double yaw = 0.002;
    int velocity;
	int wp_vel;
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
	float Time = 0;
    float error = 0;
    float K = 0;
    float Kd = 0;
    float Ki = 0;
	float offset = 0;
    // float secs = 0;
    // float nsecs = 0;    
    // float nanosec = 0;
    float pnanosec;
    float dtime = 0;
    std::vector<double> utm_;
    std::vector<double> utmt_;
    nav_msgs::Odometry own_ship_info;
	double dist_thres = 105;
	double wp_obs_thres = 5;
	double near_x_r = 10000;
	double near_y_r = 10000;
	double near_v_x_r = 0;
	double near_v_y_r = 0;
	int near_id = 0;
	double near_x_t = 10000;
	double near_y_t = 10000;
	double near_v_x_t = 0;
	double near_v_y_t = 0;
	double Dist = 10000;
	double DCPA = 10000;
	double TCPA = -1;
	bool bApp = 0;
	bool wp_call = 1;
	bool radar_call = 0;
	double psi_wp;    
	double psi_d;
	double V_norm = 3.8; // Approximately assume   600rpm, 1000rpm, 4.9, 800   4.2  about..
	double R_s = 40;
	double dThrust = 50;  // 0.1s --> 20 rp,
	double prev_thrust_l = 0.0;
	double prev_thrust_r = 0.0;

	double wp_x;
    double wp_y;
	double wp_x_temp;
	double wp_y_temp;
	double wp_x_bef;
	double wp_y_bef;

	double pin_point_x;
	double pin_point_y;

	double wayP_distance_thres = 10.0; // determine currnet obstacle is waypoint ?
	int id = 0;

	int stage = 0;

	bool wp_ok = 0;
	bool pin_flag = 1;
	bool id_init = 1;
	bool Only_turn = 0;



};

// int main(int argc, char * argv[])
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_guidance");
  GuidanceController guidance_controller;
  ros::spin();
  return 0;
}
