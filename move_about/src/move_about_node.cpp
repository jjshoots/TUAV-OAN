#include "move_lib.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "PIX");
    ros::start();
    ros::Time last_request = ros::Time::now();
    PIX HAWK;

    HAWK.set_flight_mode(1);

    while(1){
        ros::spinOnce();
        if(HAWK.request_showcurrpos){
            if(ros::Time::now() - last_request > ros::Duration(1)){
                HAWK.show_curr_pos_neat();
                HAWK.show_curr_vel_neat();
                cout<< endl;
                last_request = ros::Time::now();
            }
        }
        
        if(HAWK.PIX_state_engage){
            HAWK.send_state();
        }else{
            if(HAWK.request_sethome){
                HAWK.request_sethome = 0;
                HAWK.getHomeGeoPoint();
            }
        }
    }
}
