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



void OAN::clear_map(){
	cout<< "CLEARING OUT THE WHOLE DAMN MAP"<< endl;
	__obstacles.clear();
	__observables.clear();
	__distances.clear();
	__abs_dist.clear();
}




void OAN::check_reached(){
	float temp_length = normV2(subV2(curr_pos, goal_pos));
	if(temp_length < WAYP_REACH_CRITERIA){
		if(__waypoint.size() > 0){
			goal_pos = V2i2f(__waypoint[0]);

			//DECLARE NEW WAYPOINT
				cout<< std::setprecision(2)<< fixed;
			cout<< "NEW WAYPOINT UPDATE: "<< goal_pos.x<< ": "<< goal_pos.y<< endl;
			__waypoint.erase(__waypoint.begin());
		}else{
			if(temp_length < TARG_REACH_CRITERIA){
				cout<< "TARGET REACHED"<< endl;
				abort();	
			}
		}
	}
}



void OAN::add_to_map(V2_int new_point){
	/*
	if(__obstacles.size() > 0){
        if(sift(new_point, __obstacles)){
        	return;
        }
    }
    */

	V2_float temp;
	temp.x = 0;
	temp.y = 0;

	__obstacles.push_back(new_point);
	__distances.push_back(temp);
	__abs_dist.push_back(0);

	//int temp_i = __obstacles.size() - 1;
	//cout<< "ADD2MAP: "<< __obstacles[temp_i].x<< " : "<< __obstacles[temp_i].y<< endl;
}



void OAN::compute_distances(){
	__observables.clear();
	if(__obstacles.size() > 0){

		for(int i = 0; i < __obstacles.size(); i++){
			__distances[i].x = (float)__obstacles[i].x - curr_pos.x;
			__distances[i].y = (float)__obstacles[i].y - curr_pos.y;
			__abs_dist[i] = pow(__distances[i].x, 2.0) + pow(__distances[i].y, 2.0);
			
			//cout<< "ABS_DIST: "<< __distances[i].x<< " : "<< __distances[i].y<< endl;

			if(__abs_dist[i] < pow(OBS_RAD, 2.0)){
				__observables.push_back(__obstacles[i]);
				//cout<< "OBSERVABLES: "<<__observables.back().x<< " : "<< __observables.back().y<< endl;
			}
		}
	}
}





void OAN::find_concerns(){
	__concern[0].x = NAN;
	__concern[0].y = NAN;
	__magnitude[0] = NAN;
	__concern[1].x = NAN;
	__concern[1].y = NAN;
	__magnitude[1] = NAN;
	if(__observables.size() > 0){
		// find the first concern
		int idx = find_min_abs_dist();
		__concern[0].x = __obstacles[idx].x;
		__concern[0].y = __obstacles[idx].y;
		__magnitude[0] = sqrt(__abs_dist[idx]);
		__abs_dist[idx] = 100000;

		// if more than one obstacle, find the second concern
		if(__observables.size() > 1){
			idx = find_min_abs_dist();
			__concern[1].x = __obstacles[idx].x;
			__concern[1].y = __obstacles[idx].y;
			__magnitude[1] = sqrt(__abs_dist[idx]);
		}else{
			__concern[1].x = NAN;
			__concern[1].y = NAN;
			__magnitude[1] = NAN;
		}
	}
}



int OAN::find_min_abs_dist(){
	float mag_temp = 100000;
	int index = -1;
	if(__abs_dist.size() > 0){

		for(int i = 0; i < __abs_dist.size(); i++){
			if(__abs_dist[i] < mag_temp){
				index = i;
				mag_temp = __abs_dist[i];
			}
		}
	}
	return index;
}