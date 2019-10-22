#pragma once

#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandHome.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

class OAN{
public:
	OAN();
	~OAN();


	//CUSTOM STRUCTS
	struct V2_int{
		int x;
		int y;
		bool operator==(const V2_int &r) const{
			return ((r.x == x) && (r.y == y));
		}
	};

	struct V2_float{
		float x;
		float y;
	};

	// for velocity, please maintian NED
	struct V3_float{
		float x;
		float y;
		float z;
	};



	//FUNCTIONS
	void add_to_map(V2_int new_point);
	void map_main_loop();
	void heading_main_loop();
	void nav_main_loop();
	void send_it();
	void abort();


	bool RUN_OAN = 0;



private:
	//refer to oan_lib_base.cpp
	V2_float addV2(V2_float first, V2_float second);
	V2_float subV2(V2_float first, V2_float second);
	V2_float mulV2(V2_float first, V2_float second);
	V2_float divV2(V2_float first, V2_float second);
	V2_float emulV2(float first, V2_float second);
	V2_float edivV2(V2_float first, float second);
	V2_float V2i2f(V2_int val);
	V2_int V2f2i(V2_float val);
	float normV2(V2_float tonorm);
	void reset_timer();
	void stop_drone();
	void resetOrigin();



	//PARAMETERS
	const float LOOP_RATE = 10;
	const float OBS_RAD = 5;
	const float PROX = 3.0;
	const float MAX_VEL = 3;
	const float SNAP_GAIN = 0.1;
	const float JERK_GAIN = 0.1;
	const float ACCEL_GAIN = 0.05;
	const float VEL_GAIN = 0.05;
	const float VERT_VEL_GAIN = 0.8;
	const float MAX_RISE_VEL = -1.0;
	const float MAX_DESC_VEL = 0.5;
	const float WAYP_REACH_CRITERIA = 3; 	//distance to say we reached waypoint
	const float TARG_REACH_CRITERIA = 1; 	//distance to say we reached waypoint
	const float GET_STUCK_INTERVAL = 5;
	const float GET_STUCK_SIZE = GET_STUCK_INTERVAL * LOOP_RATE;
	const float STUCK_VELOCITY_THRESHOLD = 0.5;
	const int MAX_X = 100;
	const int MAX_Y = 100;	//assumed map sizes, note that 100 means +100, -100 range
	const int MAP_INCREMENT_CHECK = 5; 		//how much must map size increase before we perform SC2 again

	//MAP DATA
	vector<V2_int> __obstacles;
	vector<V2_int> __observables;
	vector<V2_float> __distances;
	vector<float> __abs_dist;


	//HEADING DATA
	V3_float curr_pos_offset = {0};
	V2_float curr_pos = {0};
	V2_float goal_pos = {0};
	V2_float target_pos = {0};
	float height_target = {0};
	float height_current = {0};

	V2_float __concern[2] = {0};
	float __magnitude[2] = {0};

	V2_float __initial_heading[2];
	V2_float __heading_raw = {0};

	float __speed_magnitude = 0;
	V2_float __vel_raw = {0};
	V2_float __accel_raw = {0};
	V2_float __jerk_raw = {0};
	V2_float __snap_raw = {0};
	V2_float __crackle_raw = {0};

	float __RoC_yaw = 0;
	float __prev_yaw = 0;

	// V2_float __prev_pos_raw = {0};
	V2_float __prev_vel_raw = {0};
	V2_float __prev_accel_raw = {0};
	V2_float __prev_jerk_raw = {0};

	V2_float __vel_target = {0};
	V3_float __vel_target_with_z = {0};

	//nav data
	vector<V2_int> __waypoint;
	vector<V2_int> __path;
	vector<float> __moving_window;
	float __moving_window_total = 0;
	bool __counted_total = 0;
	vector<V2_int> __nodes;
	int SC2_map_size = 0;

	//other data
	bool __local_targ = 0;


	//refer to oan_lib_map.cpp
	void check_reached();
	int find_min_abs_dist();
	void compute_distances();
	void find_concerns();
	void clear_map();

	//refer to oan_lib_heading.cpp
	V2_float find_initial_heading(V2_float concern);
	void normalize_heading();
	void dynamics();
	float solve_z_vel();

	//refer to oan_lib_nav.cpp
	bool stuck_check1();
	bool stuck_check2();
	void form_nodes_set();
	bool sift(const V2_int target, const vector<V2_int> &set);
	int locate(const V2_int target, const vector<V2_int> &set);
	void heck_yes_a_star();
	int find_min_idx(vector<float> &data);
	int find_max_idx(vector<float> &data);
	void reconstruct_path(V2_int CURRENT, vector<V2_int> &CAME_FROM_LEFT, vector<V2_int> &CAME_FROM_RIGHT, V2_int START);
	float near_obstacles(V2_int data);
	void split_merge();



	ros::NodeHandle _nh;


	//map callbacks
	void __curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber curr_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 5, boost::bind(&OAN::__curr_pos_cb, this, _1));
    geometry_msgs::PoseStamped curr_pos_raw;

    void __curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    ros::Subscriber curr_vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 5, boost::bind(&OAN::__curr_vel_cb, this, _1));
    geometry_msgs::TwistStamped curr_vel_raw;
    V2_float curr_vel;

    void __goal_pos_cb(const geometry_msgs::Point::ConstPtr& msg);
    ros::Subscriber goal_pos_sub = _nh.subscribe<geometry_msgs::Point>
            ("PIX/OAN/target_pos", 3, boost::bind(&OAN::__goal_pos_cb, this, _1));
    geometry_msgs::Point goal_pos_raw;



    //obstacle callbacks
    void __obs_location_cb(const geometry_msgs::Point::ConstPtr& msg);
    ros::Subscriber obs_location_sub = _nh.subscribe<geometry_msgs::Point>
            ("PIX/OAN/obs_location", 3, boost::bind(&OAN::__obs_location_cb, this, _1));
    geometry_msgs::Point obs_location;

    void __oan_engage_cb(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber oan_engage_sub = _nh.subscribe<std_msgs::Bool>
            ("PIX/OAN/engage", 3, boost::bind(&OAN::__oan_engage_cb, this, _1));
    std_msgs::Bool oan_engage;

    void __oan_func_cb(const std_msgs::Int16::ConstPtr& msg);
    ros::Subscriber oan_func_sub = _nh.subscribe<std_msgs::Int16>					//MOVE/MID -> OAN func
            ("PIX/OAN/oan_func", 3, boost::bind(&OAN::__oan_func_cb, this, _1));
    std_msgs::Int16 oan_func;



    //publisher
    ros::Publisher mid_state_setpoint_pub = _nh.advertise<mavros_msgs::PositionTarget>
            ("PIX/state_setpoint", 10);
    mavros_msgs::PositionTarget mid_state_setpoint;

    ros::Publisher move_func_pub = _nh.advertise<std_msgs::Int16>	//OAN -> MOVE func
            ("OAN/PIX/move_func", 3);
    std_msgs::Int16 move_func;

    //test update



    ///////////////// END HANDLERS /////////////////
    ros::Rate rate = ros::Rate(LOOP_RATE);
    ros::Time last_request = ros::Time::now();
    float TSTEP = 1/LOOP_RATE;
};


//testing some stuff here

//bla bla bla




















