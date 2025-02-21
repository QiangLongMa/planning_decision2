/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "gnss/msg/imu.hpp"
#include "gnss/msg/ins.hpp"

#include "parser.h"

namespace gnss {

class DataParser 
{
public:
  DataParser(const config::Config& config, rclcpp::Node::SharedPtr node);
  ~DataParser() {}
  bool Init();
  void ParseRawData(const uint8_t* buffer_, size_t length);

private:
  void DispatchMessage(const MessageInfo& message_info);
  void PublishInspva(const gnss::msg::Ins* message);
  void PublishCorrimu(const gnss::msg::Imu* message);
  void PublishImu(const gnss::msg::Imu* message);

  bool init_flag_ = false;
  config::Config config_;
  std::unique_ptr<Parser> data_parser_;
 
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Publisher<gnss::msg::Ins>::SharedPtr inspva_publisher_ = nullptr;
  rclcpp::Publisher<gnss::msg::Imu>::SharedPtr rawimu_publisher_ = nullptr;
  rclcpp::Publisher<gnss::msg::Imu>::SharedPtr corrimu_publisher_ = nullptr;
};

}  // namespace gnss

