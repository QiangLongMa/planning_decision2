#include <rclcpp/rclcpp.hpp>
#include "text_global.h"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("text_global"), "text_global start");

  auto node = std::make_shared<TextGlobal>();

  // 每隔10秒执行一次process
    rclcpp::Rate rate(0.1);
    while (rclcpp::ok())
    {
        node->process();
        rate.sleep();
    }
  return 0;
}
