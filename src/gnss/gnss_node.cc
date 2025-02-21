#include "rclcpp/rclcpp.hpp"

#include "stream/raw_stream.h"
#include "config/config.h"
#include<iostream>
namespace gnss {

class GnssDriverNode : public rclcpp::Node
{
public:
  GnssDriverNode(std::string name_) : Node(name_){
    RCLCPP_INFO(this->get_logger(), "%s start!", name_.c_str());
    config::Config config;
    auto raw_stream = std::make_shared<RawStream>(config, (rclcpp::Node::SharedPtr)this);
    raw_stream->Init();
    raw_stream->Start();
  }
};
} // namesapce gnss

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto gnss_driver_node = std::make_shared<gnss::GnssDriverNode>("gnss_drive_node");
  rclcpp::spin(gnss_driver_node);
  rclcpp::shutdown(); 

  return 0; 
}
   


