#include "oan_lib.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "OAN");
    ros::start();
    ros::Rate rate = ros::Rate(10.0);
    OAN navigator;
    cout<< "NAVIGATOR STARTUP"<< endl<< endl;

	while(1){
	    ros::spinOnce();
	    rate.sleep();

		if(navigator.RUN_OAN){
			
		    navigator.map_main_loop();
		    navigator.heading_main_loop();
		    navigator.nav_main_loop();

		    //check again before sending in case we ended in the mids of loops
		    if(navigator.RUN_OAN){
		    	navigator.send_it();
		    }
	    }
    }
};