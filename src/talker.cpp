#include "testing/talker.h"

void Talker::talk(int number){
    std_msgs::String msg;
    std::stringstream ss;
    ss<<"hello world ";
    ss<<number;
    msg.data = ss.str();
    pub_.publish(msg);
}

int Talker::doSomeMath(int value) {
    int next_value = 0;
    next_value = 5 + value;
    return next_value > 50 ? 0 : next_value;
}
