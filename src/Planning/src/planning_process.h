#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "ros2_path_interfaces/srv/path_interfaces.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp" // 可视化消息类型
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "toolKits.h"
#include "Calobs.h"
#include "tools/frame.h"
#include <vector>
#include <iostream>
#include <cmath>
#include "QP.h"
#include "scenario_manager.h"
#include "scenario.h"
#include "first_run.h"
#include "lane_follow.h"
#include "approaching_intersection.h"
#include "near_stop.h"
#include <unistd.h>
#include<boost/filesystem.hpp>
#include <sys/stat.h>
#include <sys/types.h>


class PlanningProcess : public rclcpp::Node
{

public:
    PlanningProcess();
    ~PlanningProcess() = default;

    bool process();

private:
    // 相应全局地图服务的回调函数，用于人机交互
    void global_path_callback(
        const std::shared_ptr<ros2_path_interfaces::srv::PathInterfaces::Request> request,
        const std::shared_ptr<ros2_path_interfaces::srv::PathInterfaces::Response> response);

    // gps的回调函数
    void gps_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 全局路径的回调函数
    void global_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 雷达的回调函数
    void lidar_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 相机的回调函数
    void cam_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 速度挡位信息的回调函数
    void speed_gears_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);

    // 局部路径生成函数
    bool get_local_path();

    // 发送障碍物信息
    void SendControlObs(std::vector<obses_sd> &obses);
    void SendHmiObs(std::vector<Eigen::VectorXd> &obses);
    void SendGlobalObses(std::vector<Eigen::VectorXd> &obses);

     // 系统控制函数
    void publish_localpath(Eigen::MatrixXd & localpath);
    void write_localpath (Eigen::MatrixXd &path);

private:
    // 智能指针对象frame_
    std::shared_ptr<Frame> frame_;
    // 服务器端
    rclcpp::Service<ros2_path_interfaces::srv::PathInterfaces>::SharedPtr global_path_service_;
    // gps 订阅数据
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_subscribe_;
    // 地图订阅数据
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_path_subscribe_;
    // 订阅雷达数据
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_LiDAR_;
    // 订阅相机数据
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cam_;
    // 订阅速度和挡位信息
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr speed_gears_subscribe_;

    /**************************发布者*********************/
    // 发布局部路径到hmi
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_to_hmi_publisher_;
    // 发布路径到控制节点
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_to_control_publisher_;
    // 可视化car，发布局部路径中路径点对应的的框
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sub_car_local_;
    // 局部路径发布到rviz
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_rviz_local_;
    // 障碍物信息发布到hmi
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_hmi_obses_;
    // 障碍物信息发布到控制节点
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_control_obses_;
    // 障碍物信息发布到全局路径，全局坐标系下的障碍物的发布
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_global_obses_;
    // 减速标志发布，向控制发布是否在减速 0为未减速 1为减速
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_control_DecelerateFlag_;
    // 停止线标志发布
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_stop_line_;

    /***************主执行函数********************/
    rclcpp::TimerBase::SharedPtr timer_getlocalpath_;
    // 系统每隔100ms执行一次局部路径生成主函数
    int Slowdowntimethreshold = 100;

   


    /************************gps信号*****************************/
    double gpsx_ = 0.0;
    double gpsy_ = 0.0;
    double gpsD_ = 0.0;
    double gpsS_ = 0.0;
    double gpsA_ = 0.0;

    // car的位置坐标
    Eigen::VectorXd car_;

    // 雷达的信息
    Eigen::MatrixXd obs_lidar_;

    /**************obs*********/
    int obs_car_globalpath_index = 0;//计算障碍物时，车辆在全局路径的编号 

    
    /**************Calobs***************/
    std::vector<obses_sd> obses_limit_SD; //在范围内的SD
    std::vector<Eigen::VectorXd> LidarcoordinatesystemObsesLimit;//障碍物的xy 在范围内的雷达坐标系下的障碍物                  
    std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit;


    Eigen::MatrixXd globalPath;

    /********************ScenarioManager智能指针对象*****************************/
    std::unique_ptr<ScenarioManager> scenario_manager_;
    ScenarioState state_;
    std::unique_ptr<Scenario> scenario_;

    /**********************局部路径***************************/
    // 本周期局部路径
    Eigen::MatrixXd optTrajxy;

    int indexinglobalpath = 0;//车辆在全局路径下的坐标 

    Eigen::MatrixXd T_obs_lidar;


    double heading_time_;





};

#endif // PLANNING_PROCESS_H_