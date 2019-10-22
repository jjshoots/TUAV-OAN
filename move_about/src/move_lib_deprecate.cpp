#include "move_lib.h"
#include <iostream>
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

using namespace std;


void PIX::send_pose(double x, double y, double z){
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    if(ros::ok()){
        local_pos_pub.publish(pose);
    }

    ros::spinOnce();
    rate.sleep();
    //cout<< pose.pose.position.x<< ":"<< pose.pose.position.y<< ":"<< pose.pose.position.z<< endl;
};



bool PIX::send_attitude_target(double thrust_val){
    if(thrust_val <= 1.0 && thrust_val >= 0.0){
        srv_thrust.thrust = thrust_val;

        if(ros::ok()){
            
            srv_attitude.header.stamp = ros::Time::now();
            srv_attitude.header.seq = __count++;
            srv_attitude.header.frame_id = 1;
            srv_attitude.pose.position.x = 0;
            srv_attitude.pose.position.y = 0;
            srv_attitude.pose.position.z = 0;
            srv_attitude.pose.orientation.x = 0;
            srv_attitude.pose.orientation.y = 0;
            srv_attitude.pose.orientation.z = 0;
            srv_attitude.pose.orientation.w = 1;
            
            attitude_target_pub.publish(srv_attitude);
            thrust_pub.publish(srv_thrust);
        }

        ros::spinOnce();
        rate.sleep();
        return 1;
    }
    return 0;    
}
