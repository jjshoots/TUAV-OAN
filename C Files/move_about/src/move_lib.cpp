#include "move_lib.h"
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


bool PIX::reset_home(){
    ros::spinOnce();

    // set new home in ros
    home.geo.latitude = GPS.latitude;
    home.geo.longitude = GPS.longitude;
    home.geo.altitude = GPS.altitude;

    // prepare new home in px4
    srv_change_home.request.latitude = GPS.latitude;
    srv_change_home.request.longitude = GPS.longitude;
    srv_change_home.request.altitude = GPS.altitude;

    if(ros::ok()){
        if( change_home.call(srv_change_home) && srv_change_home.response.success){
            ros::spinOnce();
            __home_set = 1;
            cout<< "HOME CHANGED TO: "<<
                    srv_change_home.request.latitude<< " : "<<
                    srv_change_home.request.longitude<< " : "<<
                    srv_change_home.request.altitude<< endl;
            return 1;
        }
    }
    return 0;
}



void PIX::reset_origin(){
    //if we reset origin here, ensure that OAN also reset origin
    oan_func.data = 3;
    oan_func_pub.publish(oan_func);
    ros::spinOnce();

	curr_pos_offset = curr_pos;

    cout<< "RESETTING ORIGIN by: x= "
		<< -curr_pos_offset.pose.position.y<< " | y= "
		<< -curr_pos_offset.pose.position.x<< " | z= "
		<< curr_pos_offset.pose.position.z<< endl;

}



void PIX::disp_state(){
    if(ros::ok()){
        cout<< endl;
        cout<< "--------------"<< endl;
        cout<< "--------------"<< endl;
        ros::spinOnce();
        if(current_state.armed){
            cout<< "SYSTEM IS ARMED"<< endl<< endl;
        }else{
            cout<< "SYSTEM IS DISARMED"<< endl<< endl;
        }
        cout<< current_state.mode<< " MODE"<< endl<< endl;
        
        cout<< std::setprecision(2)<< fixed;

        cout<< "POSITION TARGET:         x= "
            << (float)setpoint_target.position.y - curr_pos_offset.pose.position.y<< " | y= "
            << (float)setpoint_target.position.x - curr_pos_offset.pose.position.x<< " | z= "
            << -(float)setpoint_target.position.z + curr_pos_offset.pose.position.z<< " | yaw= "
            << (float)setpoint_target.yaw - M_PI/2<<endl;
        
        cout<< "CURRENT LOCAL POSITIION: x= "
            << curr_pos.pose.position.y - curr_pos_offset.pose.position.y<< " | y= "
            << curr_pos.pose.position.x - curr_pos_offset.pose.position.x<< " | z= "
            << -curr_pos.pose.position.z + curr_pos_offset.pose.position.z<< endl;
        
        cout<< "--------------"<< endl;
        cout<< "--------------"<< endl;
        cout<< endl;
    }
}

void PIX::disp_state_neat(){
    if(ros::ok()){
        if(current_state.armed){
            cout<< "ARMED -- ";
        }else{
            cout<< "DISARMED -- ";
        }
        cout<< current_state.mode<< endl;
    }
}


void PIX::show_curr_pos_neat(){
    cout<< std::setprecision(2)<< fixed;
    
	cout<< "CURR POS: x= "
        << curr_pos.pose.position.y - curr_pos_offset.pose.position.y<< " | y= "
        << curr_pos.pose.position.x - curr_pos_offset.pose.position.x<< " | z= "
        << -curr_pos.pose.position.z + curr_pos_offset.pose.position.z;
    cout<< "\t POS TARG: x= "
        << setpoint_target.position.y - curr_pos_offset.pose.position.y<< " | y= "
        << setpoint_target.position.x - curr_pos_offset.pose.position.x<< " | z= "
        << -setpoint_target.position.z + curr_pos_offset.pose.position.z<< " | yaw= "
        << setpoint_target.yaw - M_PI/2<<endl;
}



void PIX::show_curr_vel_neat(){
    cout<< std::setprecision(2)<< fixed;
    
    cout<< "CURR VEL: x= "
        << curr_vel.twist.linear.y<< " | y= "
        << curr_vel.twist.linear.x<< " | z= "
        << -curr_vel.twist.linear.z;
    cout<< "\t VEL TARG: x= "
        << setpoint_target.velocity.y<< " | y= "
        << setpoint_target.velocity.x<< " | z= "
        << -setpoint_target.velocity.z<< endl;
}



bool PIX::set_arm_state(bool state){
    arm_cmd.request.value = state;

    if(ros::ok()){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
            ros::spinOnce();
            return 1;
        }
    }
    return 0;
}



bool PIX::set_flight_mode(int mode){
    if(ros::ok()){
        if(mode == 1){
            set_mode.request.custom_mode = "STABILIZED";
            cout<< "STABILIZED MODE"<< endl;
        }else
        if(mode == 2){
            set_mode.request.custom_mode = "ALTCTL";
            cout<< "ALTITUDE MODE"<< endl;
        }else
        if(mode == 3){
            set_mode.request.custom_mode = "POSCTL";
            cout<< "POSCTL MODE"<< endl;
        }else
        if(mode == 4){
            set_mode.request.custom_mode = "OFFBOARD";
            cout<< "OFFBOARD MODE"<< endl;
        }else{
            cout<< "-----MODE CHANGE FAILED-----"<< endl;
            return 0;
        }

        if( set_mode_client.call(set_mode) && set_mode.response.mode_sent){
            ros::spinOnce();
            return 1;
        }
    }
    ROS_INFO_STREAM("-----MODE CHANGE FAILED-----");
    return 0;
}


bool PIX::set_takeoff(double height){
    srv_takeoff.request.altitude = height;
    if(takeoff_client.call(srv_takeoff) && srv_takeoff.response.success){
        ros::spinOnce();
        return 1;
    }
    return 0;
}



void PIX::getHomeGeoPoint(){
	__home_set = 0;
	__GPS_retrieved = 0;
    ros::start();
    ros::Time __last_request = ros::Time::now();
    cout<< "WAITING FOR HOME LOCATION"<< endl;

    while(ros::Time::now() - __last_request < ros::Duration(2)){
		reset_home();
        sleep(0.5);
        if(ros::ok() && __home_set && __GPS_retrieved){
            cout<< "Received Home Position (WGS84 datum, lat:long:alt): "<<
                    home.geo.latitude<< ":"<<
                    home.geo.longitude<< ":"<<
                    home.geo.altitude<< endl;
            return;
        }    
    }
    
    cout<< "FAILED TO SET HOME LOCATION, PROCEEDING ANYWAY..."<< endl;
}



void PIX::send_state(){
    if(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        state_pub.publish(state);
    }
    ros::spinOnce();
}



void PIX::rezero_flight(){
    mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    mid_state_setpoint.position.x = curr_pos.pose.position.x;
    mid_state_setpoint.position.y = curr_pos.pose.position.y;
    mid_state_setpoint.position.z = curr_pos.pose.position.z;
    mid_state_setpoint.yaw = M_PI/2;

    state = mid_state_setpoint;
}









