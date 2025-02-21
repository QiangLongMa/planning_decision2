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



#include <cmath>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "boost/array.hpp"

#include "data_parser.h"
#include "parser.h"
#include "../util/time_conversion.h"

namespace gnss {
DataParser::DataParser(const config::Config& config, rclcpp::Node::SharedPtr node)
  : config_(config), node_(node)
{

}

bool DataParser::Init() 
{
  inspva_publisher_ = node_->create_publisher<gnss::msg::Ins>("/sensor/gnss/inspvax", 5);
  rawimu_publisher_ = node_->create_publisher<gnss::msg::Imu>("/sensor/gnss/rawimux", 5);
  corrimu_publisher_ = node_->create_publisher<gnss::msg::Imu>("/sensor/gnss/corrimu", 5);

  data_parser_.reset(Parser::CreateParser(config_));
  if (!data_parser_)
  {
    std::cout << "Failed to create data parser.\n";
    return false;
  }

  init_flag_ = true;
  return true;
}

void DataParser::ParseRawData(const uint8_t* buffer_, size_t length)
{
  if (!init_flag_)
  {
    std::cout << "Data parser not init.\n";
    return;
  }

  data_parser_->Update(buffer_, length);

  MessageInfo message;
  do
  {
    data_parser_->GetMessages(&message);
    DispatchMessage(message);
  } while (message.type != MessageType::NONE);
}

void DataParser::DispatchMessage(const MessageInfo& message_info) 
{
  auto& message = message_info.message_ptr;
  switch (message_info.type)
  {
    case MessageType::INSPVAX:
      PublishInspva((gnss::msg::Ins*)message);
      break;
    case MessageType::CORRIMUDATAS:
      PublishCorrimu((gnss::msg::Imu*)message);
      break;
    case MessageType::RAWIMUX:
      PublishImu((gnss::msg::Imu*)message);
      break;
    case MessageType::NONE:
      return;
    default:
      break;
  }
  
}

void DataParser::PublishInspva(const gnss::msg::Ins* message)
{
  inspva_publisher_->publish(*message);
}

void DataParser::PublishImu(const gnss::msg::Imu* message)
{
  rawimu_publisher_->publish(*message);
}

void DataParser::PublishCorrimu(const gnss::msg::Imu* message)
{
  corrimu_publisher_->publish(*message);
}

}  // namespace gnss

