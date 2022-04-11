#include "slam_lib.h"


SLAM::SLAM(){

}


SLAM::~SLAM(){

}



void SLAM::__oan_func_cb(const std_msgs::Int16::ConstPtr& msg){
	oan_func = *msg;
	if(oan_func.data == 0){
		__local_targ = 0;
	}else
	if(oan_func.data == 1){
		__local_targ = 1;
	}else
	if(oan_func.data == 3){
		__hgt_offset = __height_current;
	}
}



void SLAM::__goal_pos_cb(const geometry_msgs::Point::ConstPtr& msg){
	goal_pos_raw = *msg;
	if(!__local_targ){
		__height_target = goal_pos_raw.z + __hgt_offset;
	}else{
		__height_target = goal_pos_raw.z + __height_current;
	}
}



void SLAM::__curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos_raw = *msg;
    __height_current = -curr_pos_raw.pose.position.z;
}



void SLAM::__point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
	point_cloud = *msg;
}