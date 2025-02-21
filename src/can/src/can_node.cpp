#include"can/socketcan.h"
#include <chrono>
#include <memory>
#include<ctime>
#include "rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int64_multi_array.hpp"
#include"std_msgs/msg/int32.hpp"
#include <csignal>
#include"std_msgs/msg/float32_multi_array.hpp"
#include<thread>
using namespace std;
using std::placeholders::_1;


class can_node:public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sub_brake_flag;

    rclcpp::TimerBase::SharedPtr timer_;
    socketcan* socketcan_ = nullptr;
    //std::unique_ptr<socketcan> socketcan_;  // 使用 unique_ptr 进行内存管理
    bool flag = false;
    bool exit_socketcan = true;
public:
    can_node(string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s start!", name.c_str());
        sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/control_pub",1,std::bind(&can_node::callback, this, std::placeholders::_1));
        sub_brake_flag = this->create_publisher<std_msgs::msg::Int32>("/brake_flag",5);//刹车信号发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&can_node::sendbrakeflag, this));
        socketcan_ = new  socketcan();
        //socketcan_ = std::make_unique<socketcan>();

    };
    void sendbrakeflag(){
        if (socketcan_) {
            int biaozhi = socketcan_->getbrakeflag();
            if (biaozhi < 0) return; 
            std_msgs::msg::Int32 msg;
            msg.data = biaozhi;
            RCLCPP_INFO(this->get_logger(), "Published brake flag: %d", biaozhi);
            std::cout<<"biaozhi"<<std::endl;
            sub_brake_flag->publish(msg);
        } else {
            std::cout<<"socketcan_ is null, cannot send brake flag"<<std::endl;
            RCLCPP_WARN(this->get_logger(), "socketcan_ is null, cannot send brake flag");
        }
    }
    void callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if(static_cast<int>(msg->data[4])==0) {
            if (flag) {
                if (exit_socketcan) {
                    std::cout<<"can_node exit"<<std::endl;
                    socketcan_->~socketcan();
                    //socketcan_->exit_socketcan();
                    delete socketcan_;
                    exit_socketcan = false;
                    //socketcan_=nullptr;
                }
            }
        }
        else {
            if (socketcan_!=nullptr) {
                std::cout << "socketcan_ is initialized: " << (socketcan_  != nullptr ? "Yes" : "No") << std::endl;
                flag = true;
                int speed = static_cast<int>(msg->data[0]);
                int sw_angle = static_cast<int>((msg->data[2]));
                int brake = static_cast<int>(msg->data[3]);
                unsigned char gear ;
                gear = get_gear_from_data(static_cast<int>(msg->data[1]));
                // if (static_cast<int>(msg->data[1])==0)
                // {
                //     gear = 0xB;
                // }
                // if (static_cast<int>(msg->data[1])==1)
                // {
                //     gear = 0xA;
                // }
                // if (static_cast<int>(msg->data[1])==2)
                // {
                //     gear = 0x1;
                // }
                // if (static_cast<int>(msg->data[1])==3)
                // {
                //     gear = 0x9;
                // }
                socketcan_->meeeage_all(brake,sw_angle,gear,speed);
             }
           
        }
    };
    // 获取 gear 对应的值
    unsigned char get_gear_from_data(int data) {
        switch (data)
        {
            case 0: return 0xB;
            case 1: return 0xA;
            case 2: return 0x1;
            case 3: return 0x9;
            default: return 0x0;  // 默认返回值
        }
    }

    ~can_node( ){
        socketcan_->~socketcan();
        delete socketcan_;
        //socketcan_->exit_socketcan(); 
       //delete socketcan_;
    };
};



int main(int argc, char  **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<can_node>("socketcan");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
