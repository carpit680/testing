#include "testing/talker.h"

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <thread>
#include <chrono>

// using namespace std;

class MyTestSuite : public ::testing::Test {
  public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};

TEST_F(MyTestSuite, lowValue) {  
  Talker rt;
  int initial_value = 3;
  int value = rt.doSomeMath(initial_value);
  ASSERT_EQ(value, initial_value+5) << "Value should be it's initial value plus 5";
}

TEST_F(MyTestSuite, highValue) {
  Talker rt;
  int initial_value = 49;
  int value = rt.doSomeMath(initial_value);
  ASSERT_EQ(value, 0) << "Value should be 0";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
