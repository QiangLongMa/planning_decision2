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

#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "stream.h"
#include "../parser/data_parser.h"
#include "../config/config.h"

namespace gnss {

class RawStream
{
public:
  RawStream(const config::Config& config, rclcpp::Node::SharedPtr node);
  ~RawStream();

  bool Init();

  struct Status
  {
    bool filter[Stream::NUM_STATUS] = {false};
    Stream::Status status;
  };

  void Start();

private:
  void DataSpin();
  bool Connect();
  bool Disconnect();
  bool Login();
  bool Logout();

  static constexpr size_t BUFFER_SIZE = 2048;
  static constexpr size_t READ_SIZE = 500;
  uint8_t buffer_[BUFFER_SIZE] = {0};

  std::shared_ptr<Stream> data_stream_;
  std::shared_ptr<Status> data_stream_status_;

  std::unique_ptr<std::thread> data_thread_ptr_;
  std::unique_ptr<DataParser> data_parser_ptr_;

  config::Config config_;
  rclcpp::Node::SharedPtr node_;
};

} // namespace gnss