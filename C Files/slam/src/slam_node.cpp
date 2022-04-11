#include <string>
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include "slam_lib.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "SLAM");
    ros::start();
    ros::Rate rate = ros::Rate(1.0);
	SLAM slam;
	cout<< "SLAM 2 MAP STARTUP"<< endl<< endl;

    while(1){
    	ros::spinOnce();
	    rate.sleep();
    	slam.update_map();
    }
}


