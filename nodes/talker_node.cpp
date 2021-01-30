#include <ros/ros.h>
#include "testing/talker.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    Talker rt;
    ros::Rate loop_rate(1);
    int value = 3;
    while(ros::ok()){
        rt.talk(value);
        ROS_INFO("I'm talking %d", value);
        value = rt.doSomeMath(value);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
