#include"canbus/canbus_socketcan.h"
#include <chrono>
#include <memory>
#include<ctime>
#include "rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int64_multi_array.hpp"
#include"std_msgs/msg/int16.hpp"
#include"std_msgs/msg/float32.hpp"
#include <csignal>
#include"std_msgs/msg/float32_multi_array.hpp"
#include"canbus_msg/msg/can_bus_msg.hpp"
#include<thread>
using namespace std;
using std::placeholders::_1;

class canbus_node:public rclcpp::Node
{
private:
    rclcpp::Subscription<canbus_msg::msg::CanBusMsg>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vehicle_speed;
    rclcpp::TimerBase::SharedPtr timer;
    //canbus_socketcan * canbus_socket = new canbus_socketcan();
    std::unique_ptr<canbus_socketcan> canbus_socket;
    bool flag = false;   
    bool exit_socketcan = true;
public:   
    canbus_node(string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "%s start!", name.c_str());
        sub = this->create_subscription<canbus_msg::msg::CanBusMsg>("/control_pub",1,std::bind(&canbus_node::callback, this, _1));
        pub_vehicle_speed  = this->create_publisher<std_msgs::msg::Float32>("/vehicle_speed", 1);
        timer = this->create_wall_timer(std::chrono::milliseconds((20)), std::bind(&canbus_node::speedcallback,this));
        canbus_socket = std::make_unique<canbus_socketcan>();
    };

    void callback(canbus_msg::msg::CanBusMsg::SharedPtr msg){
        canbus_socket->UpdateMessage(*msg);
    };
    void speedcallback() {
        if (canbus_socket) {
            std_msgs::msg::Float32 msg;
            msg.data = canbus_socket->getvehicle_speed();
            pub_vehicle_speed->publish(msg);
        }
    }
    ~canbus_node( ){
        canbus_socket->exit_socketcan(); 
        //delete canbus_socket;
    };
};
int main(int argc, char  **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<canbus_node>("canbus_socketcan");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
