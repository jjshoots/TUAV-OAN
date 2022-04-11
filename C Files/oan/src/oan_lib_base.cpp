#include "oan_lib.h"
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;



OAN::OAN(){
	
}

OAN::~OAN(){

};



void OAN::abort(){
	RUN_OAN = 0;
	cout<< "DISENGAGING OAN"<< endl;

	stop_drone();

    __path.clear();
    __waypoint.clear();
    __nodes.clear();
    target_pos.x = 0;

    __moving_window.clear();
    __moving_window_total = 0;

	__accel_raw = {0};
	__jerk_raw = {0};
	__snap_raw = {0};
	__crackle_raw = {0};

	__vel_target = {0};
	__vel_target_with_z = {0};

	__prev_vel_raw = {0};
	__prev_accel_raw = {0};
	__prev_jerk_raw = {0};

	return;
}


void OAN::send_it(){
	// check if something wrong
	cout<< std::setprecision(6)<< fixed;

	if(abs(__vel_target_with_z.x) > 2*MAX_VEL || abs(__vel_target_with_z.y) > 2*MAX_VEL || TSTEP < 1.0/LOOP_RATE/2.0){
		cout<< "SOMETHING WRONG, OAN ABORTED"<< endl;
		cout<< "TSTEP: "<< TSTEP<< endl;
		cout<< "VX: "<< __vel_target_with_z.x<< "\t VY"<< __vel_target_with_z.y<< endl;
		cout<< endl;
		abort();
		return;
	}

	///// PLEASE ENSURE THAT STUFF SENT HERE IS IN NED,
	///// NED -> ENU CONVERSION IS DONE IN move_lib_base.cpp
	double temp_ang = atan2(__vel_target_with_z.y, __vel_target_with_z.x);
	__RoC_yaw = abs(__prev_yaw - temp_ang);
	__prev_yaw = temp_ang;


    mid_state_setpoint.yaw = temp_ang;
    mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_PX |
                                        mavros_msgs::PositionTarget::IGNORE_PY |
                                        mavros_msgs::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    mid_state_setpoint.position.x = 0;
    mid_state_setpoint.position.y = 0;
    mid_state_setpoint.position.z = 0;

    mid_state_setpoint.velocity.x = __vel_target_with_z.x;
    mid_state_setpoint.velocity.y = __vel_target_with_z.y;
    mid_state_setpoint.velocity.z = __vel_target_with_z.z;

    mid_state_setpoint_pub.publish(mid_state_setpoint);
    ros::spinOnce();

    //we stuff the loop time calculation here
	TSTEP = (ros::Time::now() - last_request).toSec();
	last_request = ros::Time::now();
	rate.sleep();
}



void OAN::reset_timer(){
	rate.reset();

	bool timer_state = 0;
	while(!timer_state){
		cout<< "WAITING FOR TIMER TO BE OK..."<< endl;
		timer_state = rate.sleep();
	}

	for(int i = 1; i <= 3; i++){
		//get the timer running before we engage
		TSTEP = (ros::Time::now() - last_request).toSec();
		last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
	}

	__prev_vel_raw = curr_vel;//{0};
	__prev_accel_raw = {0};
	__prev_jerk_raw = {0};
	
	for(int i = 1; i <= 3; i++){
		//get the timer running before we engage
		TSTEP = (ros::Time::now() - last_request).toSec();
		last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();

		//initialuze this otherwise calculation will fail later
		//compute drone acceleration, jerk, snap
		V2_float temp_accel = edivV2(subV2(curr_vel, __prev_vel_raw), TSTEP);
		V2_float temp_jerk = edivV2(subV2(temp_accel, __prev_accel_raw), TSTEP);
		V2_float temp_snap = edivV2(subV2(temp_jerk, __prev_jerk_raw), TSTEP);

		//store for future calculations in next loop
		__prev_vel_raw = curr_vel;
		__prev_accel_raw = temp_accel;
		__prev_jerk_raw = temp_jerk;
	}
}



void OAN::stop_drone(){
	move_func.data = 1;
	move_func_pub.publish(move_func);
	ros::spinOnce();
	rate.sleep();
	ros::Duration(0.5).sleep();

	mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    //we send it minus the offset because later move_about will add back the offset
    //effectively causing position lock at current pos
    mid_state_setpoint.position.x = 0;
    mid_state_setpoint.position.y = 0;
    mid_state_setpoint.position.z = 0;

    mid_state_setpoint.velocity.x = 0;
    mid_state_setpoint.velocity.y = 0;
    mid_state_setpoint.velocity.z = 0;

    mid_state_setpoint_pub.publish(mid_state_setpoint);
    ros::spinOnce();
}



OAN::V2_float OAN::addV2(OAN::V2_float first, OAN::V2_float second){
	V2_float temp;
	temp.x = first.x + second.x;
	temp.y = first.y + second.y;
	return temp;
}



OAN::V2_float OAN::subV2(OAN::V2_float first, OAN::V2_float second){
	V2_float temp;
	temp.x = first.x - second.x;
	temp.y = first.y - second.y;
	return temp;
}



OAN::V2_float OAN::mulV2(OAN::V2_float first, OAN::V2_float second){
	V2_float temp;
	temp.x = first.x * second.x;
	temp.y = first.y * second.y;
	return temp;
}



OAN::V2_float OAN::divV2(OAN::V2_float first, OAN::V2_float second){
	V2_float temp;
	temp.x = first.x / second.x;
	temp.y = first.y / second.y;
	return temp;
}



OAN::V2_float OAN::emulV2(float first, OAN::V2_float second){
	V2_float temp;
	temp.x = first * second.x;
	temp.y = first * second.y;
	return temp;
}



OAN::V2_float OAN::edivV2(OAN::V2_float first, float second){
	V2_float temp;
	temp.x = first.x / second;
	temp.y = first.y / second;
	return temp;
}




float OAN::normV2(OAN::V2_float tonorm){
	float temp;
	temp = sqrt(pow(tonorm.x, 2) + pow(tonorm.y, 2));
	return temp;
}



OAN::V2_float OAN::V2i2f(OAN::V2_int val){
	V2_float temp;
	temp.x = val.x;
	temp.y = val.y;
	return temp;
}



OAN::V2_int OAN::V2f2i(OAN::V2_float val){
	V2_int temp;
	temp.x = round(val.x);
	temp.y = round(val.y);
	return temp;
}



void OAN::__curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos_raw = *msg;
    //INPUT CONVERSION OF ENU -> NED SINCE RAW DATA IS DIRECTLY FROM MAVROS
    curr_pos.x = curr_pos_raw.pose.position.y;
    curr_pos.y = curr_pos_raw.pose.position.x;
    height_current = -curr_pos_raw.pose.position.z;
}



void OAN::__curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    curr_vel_raw = *msg;
    //INPUT CONVERSION OF ENU -> NED SINCE RAW DATA IS DIRECTLY FROM MAVROS
    curr_vel.x = curr_vel_raw.twist.linear.y;
    curr_vel.y = curr_vel_raw.twist.linear.x;
}



void OAN::__goal_pos_cb(const geometry_msgs::Point::ConstPtr& msg){
	goal_pos_raw = *msg;

	if(!__local_targ){
		goal_pos.x = goal_pos_raw.x + curr_pos_offset.x;
		goal_pos.y = goal_pos_raw.y + curr_pos_offset.y;
		target_pos.x = goal_pos.x;
		target_pos.y = goal_pos.y;
		height_target = goal_pos_raw.z + curr_pos_offset.z;
		cout<< "GLOBAL ";
	}else{
		goal_pos.x = goal_pos_raw.x + curr_pos.x;
		goal_pos.y = goal_pos_raw.y + curr_pos.y;
		target_pos.x = goal_pos.x;
		target_pos.y = goal_pos.y;
		height_target = goal_pos_raw.z + height_current;
		cout<< "LOCAL ";
	}

	cout<< std::setprecision(2)<< fixed;

	cout<< "GOAL POSITION AT: "<< goal_pos.x - curr_pos_offset.x<< " : "<< goal_pos.y - curr_pos_offset.y<< " : "<< height_target - curr_pos_offset.z<< endl;
}



void OAN::__obs_location_cb(const geometry_msgs::Point::ConstPtr& msg){
	obs_location = *msg;
	obs_location.x += curr_pos_offset.x;
	obs_location.y += curr_pos_offset.y;

	V2_int temp;
	temp.x = round(obs_location.x);
	temp.y = round(obs_location.y);
	add_to_map(temp);

	cout<< std::setprecision(2)<< fixed;

	cout<< "ADDING OBSTACLE AT: "<< temp.x - curr_pos_offset.x<< " : "<< temp.y - curr_pos_offset.y<< endl;
}



void OAN::__oan_engage_cb(const std_msgs::Bool::ConstPtr& msg){
	oan_engage = *msg;

	if(oan_engage.data){
		reset_timer();

		RUN_OAN = 1;
		cout<< "ENGAGING OAN"<< endl;
	}else{
		RUN_OAN = 0;
		cout<< "DISENGAGING OAN"<< endl;
		abort();
	}
	
}



void OAN::resetOrigin(){
    ros::spinOnce();

	curr_pos_offset.x = curr_pos.x;
	curr_pos_offset.y = curr_pos.y;
	curr_pos_offset.z = height_current;

	cout<< std::setprecision(2)<< fixed;

    cout<< "RESETTING ORIGIN by: x= "
		<< -curr_pos_offset.x<< " | y= "
		<< -curr_pos_offset.y<< " | z= "
		<< -curr_pos_offset.z<< endl;

}



void OAN::__oan_func_cb(const std_msgs::Int16::ConstPtr& msg){
	oan_func = *msg;
	if(oan_func.data == 0){
		__local_targ = 0;
	}else
	if(oan_func.data == 1){
		__local_targ = 1;
	}else
	if(oan_func.data == 3){
		resetOrigin();
	}else
	if(oan_func.data == 4){
		clear_map();
	}
}


