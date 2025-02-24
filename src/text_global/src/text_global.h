#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <fstream>

class TextGlobal : public rclcpp::Node
{
public:
    TextGlobal();
    ~TextGlobal() = default;
    bool process();

private:
    // 发布全局路徑
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_publisher_;
};
