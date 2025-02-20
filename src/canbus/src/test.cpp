#include <chrono>
#include <memory>
#include<ctime>
#include "rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int64_multi_array.hpp"
#include"std_msgs/msg/float32_multi_array.hpp"
#include <csignal>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include"canbus_msg/msg/can_bus_msg.hpp"
using namespace std;

class test :public rclcpp::Node
{
    rclcpp::Publisher<canbus_msg::msg::CanBusMsg>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    test(string name) :Node(name){
        pub = this->create_publisher<canbus_msg::msg::CanBusMsg>("/control_pub",10);
        int i =0;
        while (1){
            if(i < 50000){
                canbus_msg::msg::CanBusMsg msg;
                msg.control_steer = 90;
                msg.control_steer_rate = 2;
                msg.control_throttle = 0;
                msg.control_brake = 0;
                msg.control_gear = 1;
                msg.left_light = 0;
                msg.right_light = 0;
                msg.mode_enable = 1;
                msg.steer_enable = 1;
                msg.gear_enable = 1;
                msg.throttle_enable = 0;
                msg.brake_enable = 0;
                msg.light_enable = 0;
                pub->publish(msg);
            } else {
                break;
            }
            
        }
    };
};
int main(int argc, char  **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test>("test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}