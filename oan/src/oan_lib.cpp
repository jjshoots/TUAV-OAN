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




//REFER oan_lib_map.cpp
void OAN::map_main_loop(){
	ros::spinOnce();
	check_reached();
	compute_distances();
	find_concerns();
	// cout<< __concern[0].x<< " : "<< __concern[0].y<< " : "<< __magnitude[0]<< endl;
	// cout<< __concern[1].x<< " : "<< __concern[1].y<< " : "<< __magnitude[1]<< endl<< endl;
};




//REFER oan_lib_heading.cpp
void OAN::heading_main_loop(){
	ros::spinOnce();
	//form 2 headings from 2 closest concerns
	__initial_heading[0] = find_initial_heading(__concern[0]);
	__initial_heading[1] = find_initial_heading(__concern[1]);

	for(int i = 0; i <= 1; i++){
		if(isnan(__initial_heading[i].x) || isnan(__initial_heading[i].y)){
			__initial_heading[i] = subV2(goal_pos, curr_pos);

			float HEADING_mag = normV2(__initial_heading[i]);
			__initial_heading[i] = edivV2(__initial_heading[i], HEADING_mag);
		}
	}

	// cout<< __initial_heading[0].x<< ":"<< __initial_heading[0].y<< endl;
	// cout<< __initial_heading[1].x<< ":"<< __initial_heading[1].y<< endl;

	normalize_heading();

	//cout<< __heading_raw.x<< ":"<< __heading_raw.y<< endl;

	dynamics();

	//SQUISH TOTAL VELOCITY SETPOINT INTO MAIN VEC
	__vel_target_with_z.x = __vel_target.x;
	__vel_target_with_z.y = __vel_target.y;
	__vel_target_with_z.z = solve_z_vel();

	//cout<< __vel_target_with_z.x<< ":"<< __vel_target_with_z.y<< endl;

}



void OAN::nav_main_loop(){
	ros::spinOnce();
	
	if((stuck_check1() || stuck_check2()) && normV2(subV2(curr_pos, target_pos)) > PROX){
		stop_drone();
		cout<< "WE ARE STUCK, FORMING NODES SET..."<< endl;
		form_nodes_set();
		cout<< "PERFORMING A*..."<< endl;
		heck_yes_a_star();
		cout<< "PERFORMING SPLIT AND MERGE..."<< endl;
		split_merge();
		cout<< "PATH FIND COMPLETE!"<< endl;

		//reset moving average window
		__moving_window.clear();

		//reinitialize first waypoint
		goal_pos = V2i2f(__waypoint[0]);
		__waypoint.erase(__waypoint.begin());

		cout<< "RESETTING TIMER..."<< endl;
		reset_timer();
	}
}
















