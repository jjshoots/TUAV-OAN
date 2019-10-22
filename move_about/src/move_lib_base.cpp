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

// FILE FOR HANDLERS

using namespace std;



PIX::PIX(){
    getHomeGeoPoint();
}



PIX::~PIX(){}



void PIX::__state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



void PIX::__arm_cb(const std_msgs::Bool::ConstPtr& msg){
    mid_arm_cmd = *msg;
    cout<< "ARM STATE FROM MIDWARE: "<< (bool)mid_arm_cmd.data<< endl;
    set_arm_state(mid_arm_cmd.data);
    disp_state_neat();
}




///// DO NOT TOUCH OR FIX THIS, REPORTS SHOULD BE DONE AS IS, MODIFY THE FEED IN
///// THIS IS THE CRUCIAL PART OF NED -> ENU CONVERSION FOR SENDING, AND
///// ENU -> NED FOR DISPLAY
void PIX::__state_setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg){
    mid_state_setpoint = *msg;

    // header stamps
    mid_state_setpoint.header.stamp = ros::Time::now();
    mid_state_setpoint.header.seq = __seq_count++;
    mid_state_setpoint.header.frame_id = 1;

    /// PERFORM NED -> ENU for linear
    double temp_x = mid_state_setpoint.position.x;
    double temp_y = mid_state_setpoint.position.y;
    double temp_z = mid_state_setpoint.position.z;

    double temp_vx = mid_state_setpoint.velocity.x;
    double temp_vy = mid_state_setpoint.velocity.y;
    double temp_vz = mid_state_setpoint.velocity.z;

    mid_state_setpoint.position.x = temp_y;
    mid_state_setpoint.position.y = temp_x;
    mid_state_setpoint.position.z = -temp_z;

    mid_state_setpoint.velocity.x = temp_vy;
    mid_state_setpoint.velocity.y = temp_vx;
    mid_state_setpoint.velocity.z = -temp_vz;

    if(!__local_pos){
        mid_state_setpoint.position.x += curr_pos_offset.pose.position.x;
        mid_state_setpoint.position.y += curr_pos_offset.pose.position.y;
        mid_state_setpoint.position.z += curr_pos_offset.pose.position.z;
    }else{
        mid_state_setpoint.position.x += curr_pos.pose.position.x;
        mid_state_setpoint.position.y += curr_pos.pose.position.y;
        mid_state_setpoint.position.z += curr_pos.pose.position.z;
    }

    //CONVERT YAW NED -> ENU
    mid_state_setpoint.yaw = -mid_state_setpoint.yaw + M_PI/2;

    //save converted stuff
    state = mid_state_setpoint;         // since we are sending state

    //when we display, do ENU -> NED

    float display_yaw = mid_state_setpoint.yaw - M_PI/2;

    /*
    if(debug_file_state){
        debugger_file   << ","<< mid_state_setpoint.position.y
                        << ","<< mid_state_setpoint.position.x
                        << ","<< -mid_state_setpoint.position.z
                        << ","<< mid_state_setpoint.velocity.y
                        << ","<< mid_state_setpoint.velocity.x
                        << ","<< -mid_state_setpoint.velocity.z
                        << ","<< display_yaw
                        << ","<< curr_pos.pose.position.y
                        << ","<< curr_pos.pose.position.x
                        << ","<< -curr_pos.pose.position.z
                        << ","<< curr_vel_raw.twist.linear.y
                        << ","<< curr_vel_raw.twist.linear.x
                        << ","<< -curr_vel_raw.twist.linear.z
                        << endl;
    }
    */

    cout<< std::setprecision(2)<< fixed;

    if(!__silence){

        if(mid_state_setpoint.type_mask ==  (mavros_msgs::PositionTarget::IGNORE_VX |
                                            mavros_msgs::PositionTarget::IGNORE_VY |
                                            mavros_msgs::PositionTarget::IGNORE_VZ |
                                            mavros_msgs::PositionTarget::IGNORE_AFX |
                                            mavros_msgs::PositionTarget::IGNORE_AFY |
                                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                                            mavros_msgs::PositionTarget::FORCE |
                                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE)){

            if(!__local_pos){
                cout<< "GLOBAL ";
            }else{
                cout<< "LOCAL ";
            }
            
            cout<< "P_TARG: x= "
            << mid_state_setpoint.position.y<< " | y= "
            << mid_state_setpoint.position.x<< " | z= "
            << -mid_state_setpoint.position.z<< " | yaw= "
            << display_yaw<< endl;
        }else
        if(mid_state_setpoint.type_mask ==  (mavros_msgs::PositionTarget::IGNORE_PX |
                                            mavros_msgs::PositionTarget::IGNORE_PY |
                                            mavros_msgs::PositionTarget::IGNORE_PZ |
                                            mavros_msgs::PositionTarget::IGNORE_AFX |
                                            mavros_msgs::PositionTarget::IGNORE_AFY |
                                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                                            mavros_msgs::PositionTarget::FORCE |
                                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE)){

            cout<< "V_TARG: x= "
            << mid_state_setpoint.velocity.y<< " | y= "
            << mid_state_setpoint.velocity.x<< " | z= "
            << -mid_state_setpoint.velocity.z<< endl;
        }
    }
}



void PIX::__state_engage_cb(const std_msgs::Bool::ConstPtr& msg){
    mid_state_engage = *msg;
    PIX_state_engage = mid_state_engage.data;

    if(state_en_disp_state && !mid_state_engage.data){
        state_en_disp_state = 0;
        cout<< "OFFBOARD DISENGAGE FROM MIDWARE"<< endl;

        //debug_file_state = 0;
        //debugger_file<< endl<< "END OFFBOARD FLIGHT SESSION"<< endl<< endl<< endl;
        //debugger_file.close();
        
        set_flight_mode(1);

    }else
    if(!state_en_disp_state && mid_state_engage.data){
        state_en_disp_state = 1;
        cout<< "OFFBOARD ENGAGE FROM MIDWARE"<< endl;

        reset_origin();
        rezero_flight();

        for(int i = 1; i <= 100; i++){
            send_state();
        }

        //debug_file_state = 1;
        //debugger_file.open("~/Desktop/flight_logger.txt");
        //debugger_file<< endl<< endl<< endl<< "START OFFBOARD FLIGHT SESSION"<< endl;
        //debugger_file<< "RPX,RPY,RPZ,RVX,RVY,RVZ,PX,PY,PZ,YAW,PX,PY,PZ,VX,VY,VZ"<< endl;

        set_flight_mode(4);
    };
}



void PIX::__home_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    home = *msg;
}



void PIX::__GPS_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    GPS = *msg;
    __GPS_retrieved = 1;
}



void PIX::__func_cb(const std_msgs::Int16::ConstPtr& msg){
    mid_func = *msg;
    
    if(mid_func.data == 1){
        request_sethome = 1;
        cout<< "HOME CHANGE REQUESTED FROM MIDWARE, WILL CHANGE ON DISARM"<< endl;
    }else
    if(mid_func.data == 2){
        disp_state();
    }else
    if(mid_func.data == 3){
        request_showcurrpos = 1;
    }else
    if(mid_func.data == 4){
        request_showcurrpos = 0;
    }else
    if(mid_func.data == 5){
        reset_origin();
    }else
    if(mid_func.data == 6){
        __local_pos = 1;
    }else
    if(mid_func.data == 7){
        __local_pos = 0;
    }else
    if(mid_func.data == 8){
        __silence = 1;
        cout<< "SILENT FEEDBACK"<< endl;
    }else
    if(mid_func.data == 9){
        __silence = 0;
        cout<< "LOUD FEEDBACK"<< endl;
    }else
    if(mid_func.data == 10){
        cout<< "HALT COMMAND RECEIVED"<< endl;
        rezero_flight();
        send_state();
    }

}



void PIX::__move_func_cb(const std_msgs::Int16::ConstPtr& msg){
    move_func = *msg;

    if(move_func.data == 0){
        __local_pos = 0;
    }else
    if(move_func.data == 1){
        __local_pos = 1;
    }
}



void PIX::__curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
}



void PIX::__curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    curr_vel = *msg;
}



void PIX::__setpoint_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg){
    setpoint_target = *msg;
}





















