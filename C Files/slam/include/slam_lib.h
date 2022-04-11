#pragma once

#include <string>
#include <iostream>
#pragma once

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

		void update_map();

	private:
		struct V2_int{
			int x;
			int y;
			bool operator==(const V2_int &r) const{
				return ((r.x == x) && (r.y == y));
			}
		};


		const int HEIGHT_RANGE = 3;


		vector<V2_int> local_map;

		// callback stuff n tings
		bool __local_targ = 0;
		float __hgt_offset = 0;
		float __height_target = 0;
		float __height_current = 0;

		int __PC_update = 0;		// last checked pointcloud size




		void __oan_func_cb(const std_msgs::Int16::ConstPtr& msg);
		void __goal_pos_cb(const geometry_msgs::Point::ConstPtr& msg);
		void __curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void __point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);



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