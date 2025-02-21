#include <rclcpp/rclcpp.hpp>
#include "planning_process.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("planning_process_main"), "planning start");

  auto node = std::make_shared<PlanningProcess>();
  if(!node->process())
  {
    RCLCPP_ERROR(rclcpp::get_logger("planning_process_main"), "planning failed!");
    rclcpp::shutdown();
    return 1;
  }

    // 执行node的proces()函数
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
