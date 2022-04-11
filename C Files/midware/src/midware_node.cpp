//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
///// EVERYTHING DONE HERE SHOULD BE DONE IN NED, NED -> ENU CONVERSION IS NOT DONE HERE /////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "midware");
    ros::NodeHandle _nh;
    ros::Rate rate = ros::Rate(20.0);

    ros::Publisher mid_arm_pub = _nh.advertise<std_msgs::Bool>
            ("PIX/arm_cmd", 2);
    std_msgs::Bool mid_arm_cmd;

    ros::Publisher mid_func_pub = _nh.advertise<std_msgs::Int16>        //MID -> everything func
            ("PIX/func", 2);
    std_msgs::Int16 mid_func;

    ros::Publisher mid_state_setpoint_pub = _nh.advertise<mavros_msgs::PositionTarget>
            ("PIX/state_setpoint", 5);
    mavros_msgs::PositionTarget mid_state_setpoint;

    ros::Publisher mid_state_engage_pub = _nh.advertise<std_msgs::Bool>
            ("PIX/state_engage", 2);
    std_msgs::Bool mid_state_engage;

    ros::Publisher oan_engage_pub = _nh.advertise<std_msgs::Bool>
            ("PIX/OAN/engage", 2);
    std_msgs::Bool oan_engage;

    ros::Publisher oan_obs_location_pub = _nh.advertise<geometry_msgs::Point>
            ("PIX/OAN/obs_location", 2);
    geometry_msgs::Point oan_obs_location;

    ros::Publisher oan_target_pos_pub = _nh.advertise<geometry_msgs::Point>
            ("PIX/OAN/target_pos", 2);
    geometry_msgs::Point oan_target_pos;

    ros::Publisher oan_func_pub = _nh.advertise<std_msgs::Int16>        //MID -> OAN func
            ("PIX/OAN/oan_func", 2);
    std_msgs::Int16 oan_func;

    ros::spinOnce();
    rate.sleep();



    while(1){
        cout<< endl<< endl<< endl;
	    // get string input
	    string input;
	    cout<< "Enter Command: ";
	    getline(cin, input);
	    
	    // split string input
	    vector<double> cmd;
	    stringstream cmd_split(input);
	    
	    // get command type
	    string cmd_type;
	    cmd_split >> cmd_type;

	    // get numbers after command type
	    double temp;
	    while(cmd_split >> temp){
		    cmd.push_back(temp);
	    }

	    // output command type
	    cout<< cmd_type<< " -- ";

	    // output command vectors
        if(cmd.size() > 0){
	        for(int i = 0; i < cmd.size() - 1; i++){
		        cout<< cmd[i]<< " : ";
	        }
            cout<< cmd[cmd.size()-1]<< endl;
        }

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////pose./////////////////////////////////////



        if(cmd_type == "FUNC"){
            cout<< "FUNCTION: ";
            if(cmd.size() != 1){
                cout<< "INVALID COMMAND"<< endl;
            }

            if(cmd[0] == 1){
                cout<< "REQUEST HOME CHANGE"<< endl;
                mid_func.data = 1;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 2){
                cout<< "REQUEST STATE DISPLAY"<< endl;
                mid_func.data = 2;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 3){
                cout<< "INTERMITTENT POSITION FEED -- ON"<< endl;
                mid_func.data = 3;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 4){
                cout<< "INTERMITTENT POSITION FEED -- OFF"<< endl;
                mid_func.data = 4;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 5){
                cout<< "RESET ORIGIN"<< endl;
                mid_func.data = 5;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 8){
                cout<< "SILENT FEEDBACK"<< endl;
                mid_func.data = 8;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else
            if(cmd[0] == 9){
                cout<< "LOUD FEEDBACK"<< endl;
                mid_func.data = 9;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();
            }else{
                cout<< "UNKNOWN FUNCTION REQUEST"<< endl;
            }

        }else
        if(cmd_type == "ARM"){
            cout<< "ARM OUT"<< endl;
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }
            mid_arm_cmd.data = 1;
            mid_arm_pub.publish(mid_arm_cmd);
            ros::spinOnce();
            rate.sleep();
        }else
        if(cmd_type == "DISARM"){
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }
            mid_arm_cmd.data = 0;
            mid_arm_pub.publish(mid_arm_cmd);
            ros::spinOnce();
            rate.sleep();
        }else
        if(cmd_type == "HALT"){
            oan_engage.data = 0;
            oan_engage_pub.publish(oan_engage);
            ros::spinOnce();
            rate.sleep();

            cout<< "HALT HALT HALT"<< endl;
            mid_func.data = 10;
            mid_func_pub.publish(mid_func);
            ros::spinOnce();
            rate.sleep();
        }else




        if(cmd_type == "POSEN"){
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }
            mid_state_engage.data = 1;
            cout<< "ENGAGE OFFBOARD MODE"<< endl;
            mid_state_engage_pub.publish(mid_state_engage);
            ros::spinOnce();
            rate.sleep();
        }else
        if(cmd_type == "POSDISEN"){
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }
            mid_state_engage.data = 0;
            cout<< "DISENGAGE OFFBOARD MODE"<< endl;
            mid_state_engage_pub.publish(mid_state_engage);
            ros::spinOnce();
            rate.sleep();
        }else



        if(cmd_type == "POSGLOBAL"){        // follow global position, yaw follows global ned
            if(cmd.size() != 4){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                mid_func.data = 7;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();

                mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                                    mavros_msgs::PositionTarget::FORCE |
                                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
                mid_state_setpoint.position.x = cmd[0];
                mid_state_setpoint.position.y = cmd[1];
                mid_state_setpoint.position.z = cmd[2];
                mid_state_setpoint.yaw = (cmd[3]/180.0*M_PI);

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "POSGLOBAL_YAWFOLL"){        // follow global position, yaw follows velocity heading
            if(cmd.size() != 3){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                mid_func.data = 7;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();

                double temp_ang = atan2(cmd[1] - mid_state_setpoint.position.y,
                                        cmd[0] - mid_state_setpoint.position.x);
                mid_state_setpoint.yaw = temp_ang;
                mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                                    mavros_msgs::PositionTarget::FORCE |
                                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
                mid_state_setpoint.position.x = cmd[0];
                mid_state_setpoint.position.y = cmd[1];
                mid_state_setpoint.position.z = cmd[2];

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "POSLOCAL"){        // follow global position, yaw follows global ned
            if(cmd.size() != 4){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                mid_func.data = 6;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();

                mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                                    mavros_msgs::PositionTarget::FORCE |
                                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
                mid_state_setpoint.position.x = cmd[0];
                mid_state_setpoint.position.y = cmd[1];
                mid_state_setpoint.position.z = cmd[2];
                mid_state_setpoint.yaw = (cmd[3]/180.0*M_PI);

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "POSLOCAL_YAWFOLL"){        // follow global position, yaw follows velocity heading
            if(cmd.size() != 3){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                mid_func.data = 6;
                mid_func_pub.publish(mid_func);
                ros::spinOnce();
                rate.sleep();

                double temp_ang = atan2(cmd[1] - mid_state_setpoint.position.y,
                                        cmd[0] -mid_state_setpoint.position.x);
                mid_state_setpoint.yaw = temp_ang;
                mid_state_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                mid_state_setpoint.type_mask =      mavros_msgs::PositionTarget::IGNORE_VX |
                                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                                    mavros_msgs::PositionTarget::FORCE |
                                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
                mid_state_setpoint.position.x = cmd[0];
                mid_state_setpoint.position.y = cmd[1];
                mid_state_setpoint.position.z = cmd[2];

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "VELGLOBAL"){        // follow global position, yaw follows global ned
            if(cmd.size() != 4){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                mid_state_setpoint.yaw = mid_state_setpoint.yaw;
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

                mid_state_setpoint.velocity.x = cmd[0];
                mid_state_setpoint.velocity.y = cmd[1];
                mid_state_setpoint.velocity.z = cmd[2];
                mid_state_setpoint.yaw = (cmd[3]/180.0*M_PI);

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "VELGLOBAL_YAWFOLL"){        // follow global position, yaw follows velocity heading
            if(cmd.size() != 3){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                double temp_ang = atan2(cmd[1], cmd[0]);
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
    
                mid_state_setpoint.velocity.x = cmd[0];
                mid_state_setpoint.velocity.y = cmd[1];
                mid_state_setpoint.velocity.z = cmd[2];

                mid_state_setpoint_pub.publish(mid_state_setpoint);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "OAN_TARGET_GLOBAL"){
            if(cmd.size() != 3){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                oan_func.data = 0;
                oan_func_pub.publish(oan_func);
                ros::spinOnce();
                rate.sleep();
                ros::Duration(1.0).sleep();

                oan_target_pos.x = cmd[0];
                oan_target_pos.y = cmd[1];
                oan_target_pos.z = cmd[2];

                oan_target_pos_pub.publish(oan_target_pos);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "OAN_TARGET_LOCAL"){
            if(cmd.size() != 3){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                oan_func.data = 1;
                oan_func_pub.publish(oan_func);
                ros::spinOnce();
                rate.sleep();
                ros::Duration(1.0).sleep();

                oan_target_pos.x = cmd[0];
                oan_target_pos.y = cmd[1];
                oan_target_pos.z = cmd[2];

                oan_target_pos_pub.publish(oan_target_pos);
                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "OAN_ADD_OBS"){
            if(cmd.size() != 2){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                oan_obs_location.x = cmd[0];
                oan_obs_location.y = cmd[1];
                oan_obs_location_pub.publish(oan_obs_location);
                ros::spinOnce();
                rate.sleep();
                ros::Duration(1.0).sleep();
            }
        }else
        if(cmd_type == "OAN_CLEAR_OBS"){
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }else{
                oan_func.data = 4;
                oan_func_pub.publish(oan_func);

                ros::spinOnce();
                rate.sleep();
            }
        }else
        if(cmd_type == "OANEN"){
            if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }

            oan_engage.data = 1;
            cout<< "ENGAGING OBSTACLE AVOIDANCE AND NAVIGATION COMPONENT"<< endl;
            oan_engage_pub.publish(oan_engage);
            ros::spinOnce();
            rate.sleep();
        }else
        if(cmd_type == "OANDISEN"){
             if(cmd.size() != 0){
                cout<< "INVALID COMMAND"<< endl;
            }
            oan_engage.data = 0;
            cout<< "DISENGAGING OBSTACLE AVOIDANCE AND NAVIGATION COMPONENT"<< endl;
            oan_engage_pub.publish(oan_engage);
            ros::spinOnce();
            rate.sleep();
        }
        else{cout<< "INVALID COMMAND"<< endl; }

        cout<< endl<< endl<< endl;
    }
}














