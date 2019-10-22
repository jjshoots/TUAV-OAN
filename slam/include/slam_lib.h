#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class SLAM{
	public:
		SLAM();
		~SLAM();

	private:
		struct V2_int{
			int x;
			int y;
			bool operator==(const V2_int &r) const{
				return ((r.x == x) && (r.y == y));
			}
		};



		bool __local_targ = 0;
		float hgt_offset = 0;
		float height_target = 0;
		float height_current = 0;




		void __oan_func_cb(const std_msgs::Int16::ConstPtr& msg){
			oan_func = *msg;
			if(oan_func.data == 0){
				__local_targ = 0;
			}else
			if(oan_func.data == 1){
				__local_targ = 1;
			}else
			if(oan_func.data == 3){
				hgt_offset = height_current;
			}
		}
		void __goal_pos_cb(const geometry_msgs::Point::ConstPtr& msg){
			goal_pos_raw = *msg;
			if(!__local_targ){
				height_target = goal_pos_raw.z + hgt_offset;
			}else{
				height_target = goal_pos_raw.z + height_current;
			}
		}
		void __curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
		    curr_pos_raw = *msg;
		    height_current = -curr_pos_raw.pose.position.z;
		}
		void __point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
			point_cloud = *msg;
		}




	    ros::NodeHandle _nh;
	    ros::Rate rate = ros::Rate(20.0);

	    ros::Publisher oan_obs_location_pub = _nh.advertise<geometry_msgs::Point>
	            ("PIX/OAN/obs_location", 2);
	    geometry_msgs::Point oan_obs_location;

	    ros::Subscriber goal_pos_sub = _nh.subscribe<geometry_msgs::Point>
	            ("PIX/OAN/target_pos", 3, boost::bind(&SLAM::__goal_pos_cb, this, _1));
	    geometry_msgs::Point goal_pos_raw;

	    ros::Subscriber oan_func_sub = _nh.subscribe<std_msgs::Int16>					//MOVE/MID -> OAN func
	            ("PIX/OAN/oan_func", 3, boost::bind(&SLAM::__oan_func_cb, this, _1));
	    std_msgs::Int16 oan_func;

        ros::Subscriber curr_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>
	            ("mavros/local_position/pose", 5, boost::bind(&SLAM::__curr_pos_cb, this, _1));
	    geometry_msgs::PoseStamped curr_pos_raw;

	    ros::Subscriber point_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>
	            ("orb_slam2_mono/map_points", 5, boost::bind(&SLAM::__point_cloud_cb, this, _1));
	    sensor_msgs::PointCloud2 point_cloud;
};