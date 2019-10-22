#pragma once

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ctime>
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

class PIX{
public:
    PIX();
    ~PIX();

    void getHomeGeoPoint();

    bool set_arm_state(bool state);                 // set arm state
    bool set_flight_mode(int mode);                 // set flight modes
    bool set_takeoff(double height);

    void send_pose(double x, double y, double z);    // send local position to drone
    void send_state();
    bool send_attitude_target(double thrust_val);   // set thrust const
    
    void disp_state();  
    void disp_state_neat();
    void show_curr_pos_neat();    
    void show_curr_vel_neat();                  

    bool PIX_state_engage = 0;

    bool state_en_disp_state = 0;
    bool request_sethome = 0;
    bool request_showcurrpos = 0;

private:
    bool reset_home();
    void reset_origin();
    void rezero_flight();

    unsigned long __count = 1;
    bool __home_set = 0;
    bool __GPS_retrieved = 0;
    bool __silence = 1;

    int __seq_count = 1;

    bool __local_pos = 0;

    //ofstream debugger_file;
    //debug_file_state = 0;

    geometry_msgs::PoseStamped curr_pos_offset;

    ///////////////// HANDLERS /////////////////

    ros::NodeHandle _nh;

    ros::ServiceClient arming_client = _nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    mavros_msgs::SetMode set_mode;

    ros::ServiceClient takeoff_client = _nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;

    ros::ServiceClient land_client = _nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;

    ros::ServiceClient change_home = _nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/set_home");
    mavros_msgs::CommandHome srv_change_home;
    


/////////////////
/////////////////
// DEPRECATE THIS
/////////////////
/////////////////
    ros::Publisher local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    geometry_msgs::PoseStamped pose;

    ros::Publisher attitude_target_pub = _nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher thrust_pub = _nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
    geometry_msgs::PoseStamped srv_attitude;
    mavros_msgs::Thrust srv_thrust;

/////////////////
/////////////////
// DEPRECATE THIS
/////////////////
/////////////////

    ros::Publisher state_pub = _nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    mavros_msgs::PositionTarget state;

    ros::Publisher oan_func_pub = _nh.advertise<std_msgs::Int16>            //MOVE/MID -> OAN func
            ("PIX/OAN/oan_func", 2);
    std_msgs::Int16 oan_func;



    void __state_cb(const mavros_msgs::State::ConstPtr& msg);
    ros::Subscriber state_sub = _nh.subscribe<mavros_msgs::State>
            ("mavros/state", 3, boost::bind(&PIX::__state_cb, this, _1));
    mavros_msgs::State current_state;

    void __arm_cb(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber arm_sub = _nh.subscribe<std_msgs::Bool>
            ("PIX/arm_cmd", 3, boost::bind(&PIX::__arm_cb, this, _1));
    std_msgs::Bool mid_arm_cmd;

    void __func_cb(const std_msgs::Int16::ConstPtr& msg);
    ros::Subscriber func_sub = _nh.subscribe<std_msgs::Int16>                       //MID -> everything func
            ("PIX/func", 5, boost::bind(&PIX::__func_cb, this, _1));
    std_msgs::Int16 mid_func;

    void __move_func_cb(const std_msgs::Int16::ConstPtr& msg);
    ros::Subscriber move_func_sub = _nh.subscribe<std_msgs::Int16>                  //OAN -> MOVE func
            ("OAN/PIX/move_func", 5, boost::bind(&PIX::__move_func_cb, this, _1));
    std_msgs::Int16 move_func;

    void __state_setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    ros::Subscriber state_setpoint_sub = _nh.subscribe<mavros_msgs::PositionTarget>
            ("PIX/state_setpoint", 3, boost::bind(&PIX::__state_setpoint_cb, this, _1));
    mavros_msgs::PositionTarget mid_state_setpoint;

    void __state_engage_cb(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber state_engage_sub = _nh.subscribe<std_msgs::Bool>
            ("PIX/state_engage", 3, boost::bind(&PIX::__state_engage_cb, this, _1));
    std_msgs::Bool mid_state_engage;

    void __home_cb(const mavros_msgs::HomePosition::ConstPtr& msg);
    ros::Subscriber home_sub = _nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 3, boost::bind(&PIX::__home_cb, this, _1));
    mavros_msgs::HomePosition home;

    void __GPS_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    ros::Subscriber GPS_sub = _nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 3, boost::bind(&PIX::__GPS_cb, this, _1));
    sensor_msgs::NavSatFix GPS;
    
    void __curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber curr_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 3, boost::bind(&PIX::__curr_pos_cb, this, _1));
    geometry_msgs::PoseStamped curr_pos;

    void __curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    ros::Subscriber curr_vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 5, boost::bind(&PIX::__curr_vel_cb, this, _1));
    geometry_msgs::TwistStamped curr_vel;

    void __setpoint_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    ros::Subscriber setpoint_target_sub = _nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 3, boost::bind(&PIX::__setpoint_target_cb, this, _1));
    mavros_msgs::PositionTarget setpoint_target;

    ///////////////// END HANDLERS /////////////////
    ros::Rate rate = ros::Rate(50.0);
    ros::Time last_request = ros::Time::now();

};
