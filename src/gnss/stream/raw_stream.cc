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

#include "raw_stream.h"

#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "stream.h"

namespace gnss {

RawStream::RawStream(const config::Config& config, rclcpp::Node::SharedPtr node)
    : config_(config), node_(node)
{
  data_parser_ptr_.reset(new DataParser(config_, node_));
}

RawStream::~RawStream()
{
  this->Disconnect();
  if (data_thread_ptr_ != nullptr && data_thread_ptr_->joinable())
  {
    data_thread_ptr_->join();
  }
}

bool RawStream::Init()
{
  assert(data_parser_ptr_ != nullptr);
  if (!data_parser_ptr_->Init())
  {
    std::cout << "Init data parser failed.\n";
    return false;
  }

  // create serial_stream
  Stream* s = nullptr;
  s = Stream::create_serial(config_.device, config_.baud_rate);
  if (s == nullptr)
  {
    std::cout << "Failed to create data stream.\n";
    return false;
  }
  data_stream_.reset(s);

  Status *status = new Status();
  if (!status)
  {
    std::cout << "Failed to create data stream status.\n";
    return false;
  }
  data_stream_status_.reset(status);

  // connect and login
  if (!Connect())
  {
    std::cout << "gnss driver connect failed.\n";
    return false;
  }

  return true;
}

void RawStream::Start()
{
  data_thread_ptr_.reset(new std::thread(&RawStream::DataSpin, this));
}

bool RawStream::Connect()
{
  if (data_stream_)
  {
    if (data_stream_->get_status() != Stream::Status::CONNECTED)
    {
      if (!data_stream_->Connect())
      {
        std::cout << "data stream connect failed.\n";
        return false;
      }
      data_stream_status_->status = Stream::Status::CONNECTED;
    }
  }

  return true;
}

bool RawStream::Disconnect()
{
  if (data_stream_)
  {
    if (data_stream_->get_status() == Stream::Status::CONNECTED)
    {
      if (!data_stream_->Disconnect())
      {
        std::cout << "data stream disconnect failed.\n";
        return false;
      }
    }
  }

  return true;
}

void RawStream::DataSpin()
{
  while (rclcpp::ok())
  {
    size_t length = data_stream_->read(buffer_, READ_SIZE);
    if (length > 0)
    {
      data_parser_ptr_->ParseRawData(buffer_, length);
    }
  }
}

bool RawStream::Login()
{
  return true;
}

bool RawStream::Logout()
{
  return true;
}

} // namesapce gnss