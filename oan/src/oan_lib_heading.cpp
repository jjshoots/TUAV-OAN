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




OAN::V2_float OAN::find_initial_heading(OAN::V2_float concern){
	V2_float HEADING;

	//RETURN IF THE CONCERN DOES NOT EXIST
	if(isnan(concern.x) || isnan(concern.y)){
		HEADING.x = NAN;
		HEADING.y = NAN;
		return HEADING;
	}

	V2_float temp_O2T = subV2(goal_pos, concern);
	V2_float temp_P2O = subV2(concern, curr_pos);

	float a1 = atan2(temp_O2T.x, temp_O2T.y);
	float a2 = atan2(temp_P2O.x, temp_P2O.y);

	if(a1 < 0){
		a1 = a1 + 2*M_PI;
	}

	if(a2 < 0){
		a2 = a2 + 2*M_PI;
	}

	float angle = a1 - a2;

	if(0 < angle && angle < M_PI/2){
		HEADING.x = temp_P2O.y;
		HEADING.y = -temp_P2O.x;
	}else
	if(-M_PI/2 < angle && angle < 0){
		HEADING.x = -temp_P2O.y;
		HEADING.y = temp_P2O.x;
	}else{
		HEADING = subV2(goal_pos, curr_pos);
	}

	//hyper_param here, if proximity of activation = hyper_param = x
	//then uav will stop x away from obstacle

	float proximity = normV2(temp_P2O);
	float temp_scalar = 0;
	if(proximity < PROX){
		temp_scalar = PROX / proximity;
	}

	//NORMALIZE STUFF TO BE UNIT LENGTH TO BE USED LATER
	//MIX IN OBSTACLE AVOIDANCE VECTORS ALSO
	float HEADING_mag = normV2(HEADING);
	HEADING = edivV2(HEADING, HEADING_mag);

	V2_float head_mix;
	head_mix.x = -temp_P2O.x / proximity;
	head_mix.y = -temp_P2O.y / proximity;

	HEADING.x = HEADING.x + temp_scalar * head_mix.x;
	HEADING.y = HEADING.y + temp_scalar * head_mix.y;

	HEADING_mag = normV2(HEADING);
	HEADING = edivV2(HEADING, HEADING_mag);

	return HEADING;
}



void OAN::normalize_heading(){
	if(isnan(__magnitude[0]) && ~isnan(__magnitude[1])){
		float HEADING_mag = normV2(__initial_heading[1]);
		__heading_raw = edivV2(__initial_heading[1], HEADING_mag);
	}else
	if(~isnan(__magnitude[0]) && isnan(__magnitude[1])){
		float HEADING_mag = normV2(__initial_heading[0]);
		__heading_raw = edivV2(__initial_heading[0], HEADING_mag);
	}else
	if(isnan(__magnitude[0]) && isnan(__magnitude[1])){
		__heading_raw = subV2(goal_pos, curr_pos);
		float HEADING_mag = normV2(__heading_raw);
		__heading_raw = edivV2(__heading_raw, HEADING_mag);
	}else{
		__heading_raw.x = __initial_heading[0].x/__magnitude[0] + __initial_heading[1].x/__magnitude[1];
		__heading_raw.y = __initial_heading[0].y/__magnitude[0] + __initial_heading[1].y/__magnitude[1];

		float HEADING_mag = normV2(__heading_raw);
		__heading_raw = edivV2(__heading_raw, HEADING_mag);
	}
}



void OAN::dynamics(){
	float des_speed = MAX_VEL;
	float temp_length, temp_length1, temp_length2;

	// if an obstacle exists...
	if(!isnan(__concern[0].x) && !isnan(__concern[0].y)){
		V2_float temp_P2O = subV2(curr_pos, __concern[0]);
		V2_float temp_P2T = subV2(curr_pos, target_pos);

		//proximity to closest obstacle
		temp_length1 = normV2(temp_P2O);
		//proximity to target
		temp_length2 = normV2(temp_P2T);
		
		//obtain the minimum of the two
		temp_length = min(temp_length1, temp_length2);
		//our target speed is based on the proximity to the nearest obstacle, or the target
		//whichever is closer

		if((temp_P2O.x * temp_P2T.x + temp_P2O.y * temp_P2T.y) < 0){
			des_speed = MAX_VEL / (1 + exp( 2*(-temp_length + PROX) ) );
		}
	}else{
		//if the obstacle does not exist...
		//find the distance to the target
		temp_length2 = normV2(subV2(curr_pos, target_pos));

		//if the distance to target is more than a threshold, we 
		//just let the target speed be the target speed, ie: no slow down needed
		if(temp_length2 > PROX){
			des_speed = MAX_VEL;
		}else{
			//if the distance to target is less than the treshold, we
			//let the target speed be based on the proximity to target
			//allowing slowdown to destination
			des_speed = MAX_VEL / (1 + exp( 0.5 * (-temp_length2 + PROX) ) );
		}
	}

	__speed_magnitude = des_speed;

	//compute drone acceleration, jerk, snap
	V2_float temp_accel = edivV2(subV2(curr_vel, __prev_vel_raw), TSTEP);
	V2_float temp_jerk = edivV2(subV2(temp_accel, __prev_accel_raw), TSTEP);
	V2_float temp_snap = edivV2(subV2(temp_jerk, __prev_jerk_raw), TSTEP);

	//store for future calculations in next loop
	__prev_vel_raw = curr_vel;
	__prev_accel_raw = temp_accel;
	__prev_jerk_raw = temp_jerk;

	//calculate desired vel, accel, jerk, snap
	V2_float des_vel = emulV2(des_speed, __heading_raw);				//des vel based on allowable max speed and requried heading
	V2_float des_accel = edivV2(subV2(des_vel, curr_vel), TSTEP);
	V2_float des_jerk = edivV2(subV2(des_accel, temp_accel), TSTEP);
	V2_float des_snap = edivV2(subV2(des_jerk, temp_jerk), TSTEP);
	V2_float des_crackle = edivV2(subV2(des_snap, temp_snap), TSTEP);	//here temp denotes actual computed dynamics of drone

	__crackle_raw = des_crackle;
	__snap_raw = emulV2(TSTEP, addV2(emulV2(SNAP_GAIN, __crackle_raw), temp_snap));
	__jerk_raw = emulV2(TSTEP, addV2(emulV2(JERK_GAIN, __snap_raw), temp_jerk));
	__accel_raw = emulV2(TSTEP, addV2(emulV2(ACCEL_GAIN, __jerk_raw), temp_accel));
	__vel_raw = emulV2(TSTEP, addV2(emulV2(VEL_GAIN, __accel_raw), curr_vel));

	// __vel_target = des_vel;
	__vel_target = __vel_raw;

	//cout<< __vel_target.x<< ":"<< __vel_target.y<<endl;
	// cout<< des_speed<< ":"<< __vel_target.x<< ":"<< __vel_target.y<< endl;
}



float OAN::solve_z_vel(){
	float z_vel = VERT_VEL_GAIN * (height_target - height_current);
	
	if(z_vel < MAX_RISE_VEL){
		return MAX_RISE_VEL;
	}else
	if(z_vel > MAX_DESC_VEL){
		return MAX_DESC_VEL;
	}else{
		return z_vel;
	}
}














