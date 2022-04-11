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



bool OAN::stuck_check1(){
	if(__RoC_yaw < 5.5 && __RoC_yaw > 2.5 && __magnitude[0] < 1.2 * PROX && __moving_window.size() > 3 * LOOP_RATE){
		return 1;
	}


	if(__moving_window.size() < GET_STUCK_SIZE){
		//check if size of moving window is bigger than the size we require,
		//if it's smaller, add a value and ignore
		__moving_window.push_back(__speed_magnitude);
		__counted_total = 0;
		__moving_window_total = 0;

		return 0;
	}else{

		//if we haven't counted the total inside the moving window,
		//count it now, faster to compute this way
		if(!__counted_total){
			for(int i = 0; i < __moving_window.size(); i++){
				__moving_window_total += __moving_window[i];
			}
			__counted_total = 1;
		}


		//if it's bigger, add a value and...
		__moving_window.push_back(__speed_magnitude);
		//delete the first value from the total and...
		__moving_window_total += __moving_window.back();
		__moving_window_total -= __moving_window[0];
		//erase the first value from the moving window
		__moving_window.erase(__moving_window.begin());

		//compute the average of the moving window
		float moving_window_average;
		moving_window_average = __moving_window_total / __moving_window.size();
		// cout<< moving_window_average<< endl;

		if(moving_window_average < STUCK_VELOCITY_THRESHOLD){
			return 1;
		}
	}

	return 0;
}



bool OAN::stuck_check2(){
	if(__obstacles.size() == 0){
		return 1;
	}else{
		if(__obstacles.size() - SC2_map_size > MAP_INCREMENT_CHECK){
			SC2_map_size = __obstacles.size();
			for(int i = 0; i < __path.size(); i++){
				int idx = locate(__path[i], __obstacles);

				if(idx == -1){
					return 0;
				}else{
					if(near_obstacles(__path[idx]) > PROX){
						return 1;
					}else{
						return 0;
					}
				}
			}
		}else{
			return 0;
		}
	}
}



bool OAN::sift(const OAN::V2_int target, const vector<OAN::V2_int> &set){
	auto it = find(set.begin(), set.end(), target);
	if(it == set.end()){
		return 0;
	}else{
		return 1;
	}

	/*
	for(int i = 0; i < set.size(); i++){
		if(target.x == set[i].x){
			if(target.y == set[i].y){
				return 1;
			}
		}
		if(i == set.size() -1){
			return 0;
		}
	}
	*/
}



int OAN::locate(const OAN::V2_int target, const vector<OAN::V2_int> &set){
	auto it = find(set.begin(), set.end(), target);
	if(it == set.end()){
		return -1;
	}else{
		return std::distance(set.begin(), it);
	}

	/*
	for(int i = 0; i < set.size(); i++){
		if(target.x == set[i].x){
			if(target.y == set[i].y){
				return i;
			}
		}
		if(i == set.size() -1){
			return -1;
		}
	}
	*/
}



void OAN::form_nodes_set(){
	__nodes.clear();
	for(int i = -MAX_X; i <= MAX_X; i++){
		for(int j = -MAX_Y; j <= MAX_Y; j++){
			V2_int temp;
			temp.x = i;
			temp.y = j;
			if(!sift(temp, __obstacles)){
				__nodes.push_back(temp);
			}
		}
	}
}



int OAN::find_min_idx(vector<float> &data){
	float min_val = 100000;
	int min_idx = -1;
	if(data.size() > 0){
		for(int i = 0; i <= data.size() -1; i++){
			if(data[i] < min_val){
				min_val = data[i];
				min_idx = i;
			}
		}
	}
	return min_idx;
}



int OAN::find_max_idx(vector<float> &data){
	float max_val = 0;
	int max_idx = -1;
	if(data.size() > 0){
		for(int i = 0; i <= data.size() -1; i++){
			if(data[i] > max_val){
				max_val = data[i];
				max_idx = i;
			}
		}
	}
	return max_idx;
}



void OAN::reconstruct_path(OAN::V2_int CURRENT, vector<OAN::V2_int> &CAME_FROM_LEFT, vector<OAN::V2_int> &CAME_FROM_RIGHT, OAN::V2_int START){
	__path.clear();

	//last point is the point that we're at, ie: current point is the target_pos
	__path.push_back(CURRENT);
	cout<< "THIS IS THE NEW PATH WE'RE USING: "<< endl;
	cout<< std::setprecision(2)<< fixed;

	while(CURRENT.x != START.x || CURRENT.y != START.y){
		int idx = locate(CURRENT, CAME_FROM_LEFT);
		CURRENT = CAME_FROM_RIGHT[idx];
		//left came from right, therefore, we step backwards, ie:
		//continually find  right in left, and progress until right = START
		__path.insert(__path.begin(), CURRENT);
		cout<< CURRENT.x<< " : "<< CURRENT.y<< endl;
	}
}



float OAN::near_obstacles(OAN::V2_int data){
	float NEAR_OBSTACLES = 0;
	for(int i = -PROX; i <= PROX; i++){
		for(int j = -PROX; j <= PROX; j++){
			if(i != 0 && j != 0){
				V2_float temp_hold;
				temp_hold.x = i;
				temp_hold.y = j;
				V2_int temp_n_neighbour;
				temp_n_neighbour = V2f2i(addV2(V2i2f(data), temp_hold));

				if(sift(temp_n_neighbour, __obstacles)){
					NEAR_OBSTACLES += 1/i + 1/j;
				}
			}
		}
	}
	return NEAR_OBSTACLES;
}



void OAN::heck_yes_a_star(){
	int temp_size = __nodes.size();

	vector<float> G_SCORE(temp_size);
	vector<float> F_SCORE(temp_size);
	vector<V2_int> CAME_FROM_LEFT(temp_size);
	vector<V2_int> CAME_FROM_RIGHT(temp_size);

	CAME_FROM_LEFT = __nodes;

	V2_int temp_nan;
	temp_nan.x = INT_MAX;
	temp_nan.y = INT_MAX;

	fill(G_SCORE.begin(), G_SCORE.end(), numeric_limits<float>::infinity());
	fill(F_SCORE.begin(), F_SCORE.end(), numeric_limits<float>::infinity());
	fill(CAME_FROM_RIGHT.begin(), CAME_FROM_RIGHT.end(), temp_nan);

	vector<V2_int> steps;

	V2_int step_holder;
	step_holder.x = 1;
	step_holder.y = 0;
	steps.push_back(step_holder);
	step_holder.x = -1;
	step_holder.y = 0;
	steps.push_back(step_holder);
	step_holder.x = 0;
	step_holder.y = 1;
	steps.push_back(step_holder);
	step_holder.x = 0;
	step_holder.y = -1;
	steps.push_back(step_holder);

	V2_int START;
	START.x = round(curr_pos.x);
	START.y = round(curr_pos.y);

	V2_int END;
	END.x = round(target_pos.x);
	END.y = round(target_pos.y);


	vector<V2_int> CLOSED_SET;
	vector<V2_int> OPEN_SET;
	vector<float> OPEN_SET_DATA;
	OPEN_SET.push_back(START);
	OPEN_SET_DATA.push_back(0);

	int temp_idx = locate(START, __nodes);
	G_SCORE[temp_idx] = 0;
	F_SCORE[temp_idx] = normV2(subV2(V2i2f(START), V2i2f(END)));

	//new current point is the point on openset with the lowest value
	//as long as the open set still has stuff, we haven't explored finish
	while(OPEN_SET.size() > 0){
		int idx_OPEN = find_min_idx(OPEN_SET_DATA);
		int idx_CIN = locate(OPEN_SET[idx_OPEN], __nodes);
		V2_int CURRENT = OPEN_SET[idx_OPEN];

		if(CURRENT.x == (int)target_pos.x && CURRENT.y == (int)target_pos.y){
			cout<< "A* COMPLETE!"<< endl;
			reconstruct_path(CURRENT, CAME_FROM_LEFT, CAME_FROM_RIGHT, START);
			return;
		}

		//delete current point from open set since we are exploring it
		OPEN_SET.erase(OPEN_SET.begin() + idx_OPEN);
		OPEN_SET_DATA.erase(OPEN_SET_DATA.begin() + idx_OPEN);

		//add the current node we are about to explore into the closed set
		//so we do not come here naymore
		CLOSED_SET.push_back(CURRENT);

		//we are exploring as many steps as there are in steps vector
		for(int i = 0; i < 4; i++){
			V2_int NEIGHBOUR = V2f2i(addV2(V2i2f(CURRENT), V2i2f(steps[i])));
			float NEIGHBOUR_DATA = 0;

			//check if NEIGHBOUR is an available node
			//check if NEIGHBOUR is already explored, ie: in CLOSED_SET
			if(sift(NEIGHBOUR, __obstacles) || sift(NEIGHBOUR, CLOSED_SET)){
				continue;
			}

			int idx_NEIGHBOUR = locate(NEIGHBOUR, __nodes);

			//if NEIGHBOUR is not in OPEN_SET, we add the NEIGHBOUR to
			//the OPEN_SET
			if(!sift(NEIGHBOUR, OPEN_SET)){
				OPEN_SET.push_back(NEIGHBOUR);
				OPEN_SET_DATA.push_back(0);
			}

			//tentative gscore
			float T_G_SCORE = G_SCORE[idx_CIN] + 1;
			//if tentative gscore is more than the gscore we are currently at
			//it is pointless to explore this path, break
			if(T_G_SCORE > G_SCORE[idx_NEIGHBOUR]){
				continue;
			}

			//if we made it this far, this path is the best path for now
			//record the path we came from and move to the new step
			CAME_FROM_RIGHT[idx_NEIGHBOUR] = CURRENT;
			G_SCORE[idx_NEIGHBOUR] = T_G_SCORE;
			float heuristic = normV2(subV2(V2i2f(NEIGHBOUR), target_pos))*2 + 100 * near_obstacles(NEIGHBOUR);
			F_SCORE[idx_NEIGHBOUR] = T_G_SCORE + heuristic;

			int idx_NIO = locate(NEIGHBOUR, OPEN_SET);
			OPEN_SET_DATA[idx_NIO] = F_SCORE[idx_NEIGHBOUR];
		}
	}
}



void OAN::split_merge(){
	__waypoint.clear();
	int path_end_idx = __path.size()-1;
	int path_beg_idx = 0;
	int way_idx = 1;
	int path_idx = (path_end_idx + path_beg_idx) / 2;

	__waypoint.push_back(__path[path_beg_idx]);
	__waypoint.push_back(__path[path_end_idx]);

	while(1){
		// split path into two parts and pull out the middel point
		int temp_mid = path_idx;
		V2_int temp_mid_point = __path[temp_mid];

		// if the middle point is already in the waypoint list
		// we have already reached optimality, end the loop
		if(sift(temp_mid_point, __waypoint)){
			return;
		}

		// stuff the temp_mid_point into the part on the waypoint list where it belongs
		__waypoint.insert(__waypoint.begin() + way_idx, temp_mid_point);

		//number of straight lines formable by waypoints list
		int LINES_NUM = __waypoint.size() - 1;
		
		V2_float temp;
		temp.x = 0.0; temp.y = 0.0;
		vector<V2_float> M_C_Matrix(LINES_NUM);
		fill(M_C_Matrix.begin(), M_C_Matrix.end(), temp);
		
		vector<float> ERROR_SUM(LINES_NUM);
		fill(ERROR_SUM.begin(), ERROR_SUM.end(), 0.0);

		// index of the beginning waypoint on the line we are about to explore
		// for now it's 0 since we start the exploration from the 0th waypoint
		int idx_WOP_prev = 0;
		vector<float> SOS;
		vector<float> SOS_WOP;
		SOS.resize(__path.size()-1);
		SOS_WOP.resize(__path.size()-1);

		for(int i = 0; i <= LINES_NUM-1; i++){
			// perform y = mx + c
			//temp_del_x = temporary delta x
			float temp_del_x = (float)__waypoint[i+1].x - (float)__waypoint[i].x;

			//check if delta x = 0, because if 0, gradient is inf, so we just approximate inf
			float temp_m, temp_c;
			if(temp_del_x < 0.5){
				temp_m = numeric_limits<float>::infinity();	//infinite gradient
				temp_c = __waypoint[i].x;							//store only x value
			}else{
				temp_m = ((float)__waypoint[i+1].y - (float)__waypoint[i].y) / temp_del_x;
				temp_c = (float)__waypoint[i].y - temp_m * (float)__waypoint[i].x;
			}

			M_C_Matrix[i].x = temp_m;
			M_C_Matrix[i].y = temp_c;

			//locate end of line location index
			int idx_WOP = locate(__waypoint[i+1], __path);
			
			for(int j = idx_WOP_prev; j <= idx_WOP; j++){
				if(M_C_Matrix[i].y >= numeric_limits<float>::max()){	//by default, inf > max, so we use this convention for safety
					SOS[j] = abs((float)M_C_Matrix[i].y - (float)__path[j].x);
					SOS_WOP[j] = i;
				}else{
					float temp_eY = M_C_Matrix[i].x * (float)__path[j].x + (float)M_C_Matrix[i].y;
					SOS[j] = abs((float)__path[j].y - temp_eY);
					SOS_WOP[j] = i;
				}
			}
			idx_WOP_prev = idx_WOP;
		}

		int idx = find_max_idx(SOS);

		// if the error sum is more than 3, we can optimize further
		if(SOS[idx] > 2.0){
			way_idx = SOS_WOP[idx] + 1;
			path_idx = idx;
		}

	}
}

















