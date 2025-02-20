#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_path_interfaces/srv/path_interfaces.hpp"
#include <Eigen/Eigen>
#include "map.h"
#include<math.h>
#include<cmath>
#include"reference_line_provider.h"
#include <iostream>
#include <fstream>
#include <string>

class global_path_node : public rclcpp::Node{
    public:
        // 构造函数,有一个参数为节点名称
        global_path_node(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "%s node start!", name.c_str());
            // 创建客户端
            global_path_client_ = this->create_client<ros2_path_interfaces::srv::PathInterfaces>("global_path_srv");
            // 创建话题订阅者，订阅消息GPS, Dest
            gps_subscribe = this->create_subscription<std_msgs::msg::Float64MultiArray>("gps", 10, std::bind(&global_path_node::global_path_callback, this, std::placeholders::_1));
            dest_subscribe = this->create_subscription<std_msgs::msg::Float64MultiArray>("dest_pub", 10, std::bind(&global_path_node::dest_callback, this, std::placeholders::_1));
            // 创建话题发布者, 发布全局路径消息
            global_hmi_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("global_hmi_topic", 10);
            global_local_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("global_local_topic", 10);
            outputFile_index.open(filename_index);
        }

        void send_request(){
            // 1.等待服务端上线
            while (!global_path_client_->wait_for_service(std::chrono::seconds(1))){ //函数等待服务端上线，等待时间间隔为1秒。
                // 等待时检测rclcpp的状态
                if (!rclcpp::ok())            {
                    std::cout << "Interrupted while waiting for service..." << std::endl;
                    return;
                }
                std::cout << "Waiting for the server to go online..." << std::endl;
            }
            // 2.构造请求
            auto request = std::make_shared<ros2_path_interfaces::srv::PathInterfaces_Request>();
            request->global_request = 1;
            // 3.发送异步请求，然后等待返回，返回时调用回调函数
            global_path_client_->async_send_request(request, std::bind(&global_path_node::result_callback_, this, std::placeholders::_1));
        }

        void result_callback_(rclcpp::Client<ros2_path_interfaces::srv::PathInterfaces>::SharedFuture result_future){   
            auto response = result_future.get();
            if (response->local_response > 0){
                // 话题发送全局路径
                std::vector<double> globalPathReshape(&globalPath(0), globalPath.data() + globalPath.size());
                globalPathReshape.push_back(globalPath.cols());
                std_msgs::msg::Float64MultiArray msg;
                msg.data = globalPathReshape;
                global_local_publisher->publish(msg); //
            }
        }

        void global_path_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)    {
            x = msg->data[0];
            y = msg->data[1];
            Direction = msg->data[2];
            Speed = msg->data[3];
        }

        // sub dens_pub
        void dest_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            double dest_x = msg->data[0];
            double dest_y = msg->data[1];
            if ((dest_x<=0 && dest_y<=0) || (dest_x > 900)|| (dest_y > 780)){
                std::cout << "dest_point select error" << std::endl; 
            }else{
                Eigen::Matrix<double, 1, 2> realPosition;
                tool::imagePosition2realPosition(dest_x, dest_y, realPosition);
                endPoint << realPosition;
                // std::cout << "endPoint:" << endPoint << std::endl;
                std::cout << "dest_point has been select!" << std::endl;
                car.resize(5, 1);
                //int resInit = InitGlobalMap(x, y, Direction, Speed, endPoint, globalPath, car); // 初始化,获得全局路径+车辆位置
                if(std::abs(x)>1.0&&std::abs(y)>1.0){
                    InitGlobalMap(x, y, Direction, Speed, endPoint, globalPath, car);

                    Eigen::MatrixXd newglobalPath;
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix=globalPath ;
                    Eigen::MatrixXd& temp_matrixs = temp_matrix;
                    ReferenceLineProvider ReferenceLineProvider_;
                    bool status = ReferenceLineProvider_.SmoothReferenceLine(temp_matrixs,newglobalPath);
                    for (size_t i = 0; i < newglobalPath.cols(); i++)
                    {
                        outputFile_index<<globalPath(0,i)<<" "<<globalPath(1,i)<<" "<<globalPath(2,i)<<" "<<globalPath(3,i)<<" "
                                        <<globalPath(4,i)<<" "<<globalPath(5,i)<<" "<<globalPath(6,i)<<std::endl;
                    }
                    if (status==true){
                        std::cout<<"全局路径优化成功"<<std::endl;
                        std::vector<double> globalPathReshape(&newglobalPath(0), newglobalPath.data() + newglobalPath.size());
                        globalPathReshape.push_back(newglobalPath.cols());
                        globalPath.resize(7,newglobalPath.cols());
                        globalPath = std::move(newglobalPath);
                        std_msgs::msg::Float64MultiArray global_msg;
                        global_msg.data = globalPathReshape;
                        global_hmi_publisher->publish(global_msg);
                        this->send_request();
                    }
                    else{
                        std::cout<<"全局路径优化失败"<<std::endl;
                        std::vector<double> globalPathReshape(&globalPath(0), globalPath.data() + globalPath.size());
                        globalPathReshape.push_back(globalPath.cols());
                        std_msgs::msg::Float64MultiArray global_msg;
                        global_msg.data = globalPathReshape;
                        global_hmi_publisher->publish(global_msg);
                        this->send_request();
                    }
                    // std::vector<double> dk(globalPath.cols());
                    // //重新计算曲率 使用三点确定曲率 
                    // for (size_t i = 0; i < globalPath.cols(); ++i)
                    // {
                    //     if (i==0||i==(globalPath.cols()-1))
                    //     {
                    //         continue;
                    //     }
                    //     double a,b,c,r,B;
                    //     double cosB,sinA,dis;
                    //     c= std::sqrt(std::pow(globalPath(0,i)-globalPath(0,i-1),2)+std::pow(globalPath(1,i)-globalPath(1,i-1),2));
                    //     a =std::sqrt(std::pow(globalPath(0,i)-globalPath(0,i+1),2)+std::pow(globalPath(1,i)-globalPath(1,i+1),2));
                    //     b =std::sqrt(std::pow(globalPath(0,i-1)-globalPath(0,i+1),2)+std::pow(globalPath(1,i-1)-globalPath(1,i+1),2));
                    //     dis=  a*a + c*c - b*b;
                    //     cosB = dis/(2*a*c);//余弦定理求角度
                    //     B =acos(cosB);
                    //     double ke = 2*sin(B)/b;
                    //     dk[i]=ke;
                    // }
                    // dk[0]=dk[1]; dk[globalPath.cols()-1]=dk[globalPath.cols()-2];
                    // for (size_t i = 0; i < dk.size(); ++i)
                    // {
                    //     globalPath(4,i) = dk[i];
                    // }
                    
                }
                else{
                    std::cout << "gps is error!!!!" << std::endl;
                }
                // InitGlobalMap(x, y, Direction, Speed, endPoint, globalPath, car); 
                // std::vector<double> globalPathReshape(&globalPath(0), globalPath.data() + globalPath.size());
                // globalPathReshape.push_back(globalPath.cols());
                // std_msgs::msg::Float64MultiArray global_msg;
                // global_msg.data = globalPathReshape;
                // global_hmi_publisher->publish(global_msg);
                // this->send_request();
            }
        }

    private:
        // 声明global客户端
        rclcpp::Client<ros2_path_interfaces::srv::PathInterfaces>::SharedPtr global_path_client_;
        // 声明Gps订阅
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_subscribe;
        // 声明Dest订阅
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dest_subscribe;
        // 声明global->hmi话题发布者
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_hmi_publisher;
        // 声明global->local话题发布者
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_local_publisher;
        // GPS Data
        double Direction = 0.0;
        double Speed = 0.0;
        double x = 0.0;
        double y = 0.0;
        cv::Mat image;
        Eigen::VectorXd car;
        Eigen::Matrix<double, 1, 2> endPoint;                // destination select
        Eigen::Matrix<double, 7, Eigen::Dynamic> globalPath; // 全局路径声明x,y,v,theta,kappa,dkappa,s

        std::string filename_index = "outputxyglobal.txt";
        std::ofstream outputFile_index;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    /*创建节点的共享指针对象*/
    auto node = std::make_shared<global_path_node>("global_node");
    // node->send_request();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}