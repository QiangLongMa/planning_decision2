#include "text_global.h"

TextGlobal::TextGlobal(): Node("text_global")
{
    RCLCPP_INFO(this->get_logger(), "text_global created");
    local_path_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("global_local_topic", 10);
}

bool TextGlobal::process()
{
    std_msgs::msg::Float64MultiArray msg;
    // 读取outputxyglobal.txt 文件，文件目录为/home/ring/planning_decision2/outputxyglobal.txt
    std::ifstream file("/home/mm/BUS/bus2025.2.24/planning_decision2-dev/outputxyglobal.txt");
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file");
        return false;
    }
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<double> numbers(std::istream_iterator<double>{iss}, std::istream_iterator<double>());
        for (auto number : numbers)
        {
            msg.data.push_back(number);
        }
    }
    local_path_publisher_->publish(msg);
    return true;
}
