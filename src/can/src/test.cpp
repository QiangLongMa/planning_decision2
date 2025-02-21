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
using namespace std;

class test :public rclcpp::Node
{
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    test(string name) :Node(name){
        pub= this->create_publisher<std_msgs::msg::Float32MultiArray>("/control_pub",1);
        int i =0;
        while (1){
            
            std_msgs::msg::Float32MultiArray msg;
            msg.data.push_back(0);//speed
            msg.data.push_back(2);//gear  0 p     1  n     2 d    3   r
            msg.data.push_back(-90);//sw_angle
            msg.data.push_back(0);//brake
            // msg.data.push_back(1);//start stop
            // msg.data.push_back(0);//start stop
            // pub->publish(msg);
            if (i<500000)
            {
                msg.data.push_back(0);//start stop
                pub->publish(msg);
            }
            else if(i >= 500000 && i < 1000000)
            {
                msg.data.push_back(1);//start stop
                pub->publish(msg);
                
            }
            else{
                msg.data.push_back(0);//start stop
                pub->publish(msg);
                if (i > 1110000){
                    std::cout<<"exit test1"<<std::endl;
                    break;

                }
                
                //break;
            }
            i=i+1;
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