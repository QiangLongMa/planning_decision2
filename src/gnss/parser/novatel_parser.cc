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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <set>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "gnss/msg/imu.hpp"
#include "gnss/msg/ins.hpp"

#include "../config/config.h"
#include "../util/time_conversion.h"

#include "parser.h"
#include "parser_common.h"
#include "novatel_messages.h"

namespace gnss
{

  // Anonymous namespace that contains helper constants and functions.
  namespace
  {

    constexpr size_t BUFFER_SIZE = 256;

    // The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use
    // the index array below
    // to convert it to the orientation covariance matrix with order roll, pitch,
    // and yaw.
    constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
    static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

    template <typename T>
    constexpr bool is_zero(T value)
    {
      return value == static_cast<T>(0);
    }

    /*--------------------------------------------------------------------------
    Calculate a CRC value to be used by CRC calculation functions.
    -------------------------------------------------------------------------- */
    unsigned long CRC32Value(int i)
    {
      int j;
      unsigned long ulCRC;
      ulCRC = i;
      for (j = 8; j > 0; j--)
      {
        if (ulCRC & 1)
          ulCRC = (ulCRC >> 1) ^ 0xEDB88320L;
        else
          ulCRC >>= 1;
      }

      return ulCRC;
    }

    /*--------------------------------------------------------------------------
    Calculates the CRC-32 of a block of data all at once
    ulCount- Number of bytes in the data block
    ucBuffer- Data block
    -------------------------------------------------------------------------- */
    unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer)
    {
      unsigned long ulTemp1;
      unsigned long ulTemp2;
      unsigned long ulCRC = 0;
      while (ulCount-- != 0)
      {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xFF);
        ulCRC = ulTemp1 ^ ulTemp2;
      }

      return (ulCRC);
    }

    // CRC algorithm from the NovAtel document.
    inline uint32_t crc32_word(uint32_t word)
    {
      for (int j = 0; j < 8; ++j)
      {
        if (word & 1)
          word = (word >> 1) ^ 0xEDB88320;
        else
          word >>= 1;
      }

      return word;
    }

    inline uint32_t crc32_block(const uint8_t *buffer, size_t length)
    {
      uint32_t word = 0;
      while (length--)
      {
        uint32_t t1 = (word >> 8) & 0xFFFFFF;
        uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
        word = t1 ^ t2;
      }

      return word;
    }

  } // namespace

  class NovatelParser : public Parser
  {
  public:
    NovatelParser();
    explicit NovatelParser(const config::Config &config);
    virtual void GetMessages(MessageInfoVec *messages);
    virtual void GetMessages(MessageInfo *message);
    virtual MessageType GetMessage(MessagePtr *message_ptr);

  private:
    bool check_crc();

    MessageType PrepareMessage(MessagePtr *message_ptr);

    // The handle_xxx functions return whether a message is ready.
    bool HandleCorrImuData(const novatel::CorrImuData *imu);
    bool HandleInsPvax(const novatel::InsPvaX *pvax, uint16_t gps_week, uint32_t gps_millisecs);
    bool HandleRawImuX(const novatel::RawImuX *imu);

    double gyro_scale_ = 0.0;
    double accel_scale_ = 0.0;

    float imu_measurement_span_ = 1.0f / 125.0f;
    float imu_measurement_hz_ = 125.0f;

    float corrimudata_hz_ = 100.0f;
    float rawimu_hz_ = 125.0f;

    int imu_frame_mapping_ = 5;

    double imu_measurement_time_previous_ = -1.0;

    std::vector<uint8_t> buffer_;

    size_t header_length_ = 0;

    size_t total_length_ = 0;

    novatel::ImuType imu_type_ = novatel::ImuType::G320N;

    // -1 is an unused value.
    novatel::SolutionStatus solution_status_ = static_cast<novatel::SolutionStatus>(novatel::SolutionStatus::NONE);
    novatel::SolutionType position_type_ = static_cast<novatel::SolutionType>(novatel::SolutionType::NONE);
    novatel::SolutionType velocity_type_ = static_cast<novatel::SolutionType>(novatel::SolutionType::NONE);
    novatel::InsStatus ins_status_ = static_cast<novatel::InsStatus>(novatel::InsStatus::NONE);

    // raw_t raw_;
    //  used for observation data
    bool has_ins_stat_message_ = false;
    bool has_corr_imu_message_ = false;

    gnss::msg::Imu rawimu_;
    gnss::msg::Imu corrimu_;
    gnss::msg::Ins inspvax_;
  };

  Parser *Parser::CreateNovatel(const config::Config &config)
  {
    return new NovatelParser(config);
  }

  NovatelParser::NovatelParser()
  {
    buffer_.reserve(BUFFER_SIZE);
    imu_type_ = novatel::ImuType::G320N;
  }

  NovatelParser::NovatelParser(const config::Config &config)
  {
    buffer_.reserve(BUFFER_SIZE);
    imu_type_ = novatel::ImuType::G320N;
  }

  void NovatelParser::GetMessages(MessageInfoVec *messages)
  {
    std::set<MessageType> s;
    while (rclcpp::ok())
    {
      MessageInfo message_info;
      message_info.type = GetMessage(&message_info.message_ptr); // GetMessage() process data_/data_end_
      if (message_info.type == MessageType::NONE)
      {
        break;
      }

      // meaning ?
      if (s.find(message_info.type) == s.end())
      {
        messages->push_back(std::move(message_info));
        s.insert(message_info.type);
      }
    }
  }

  void NovatelParser::GetMessages(MessageInfo *message)
  {
    message->type = GetMessage(&message->message_ptr);
  }

  MessageType NovatelParser::GetMessage(MessagePtr *message_ptr)
  {
    if (data_ == nullptr)
    {
      return MessageType::NONE;
    }

    while (data_ < data_end_)
    {
      if (buffer_.empty())
      { // Looking for SYNC0
        if (*data_ == novatel::SYNC_0)
        {
          buffer_.push_back(*data_);
        }
        ++data_;
      }
      else if (buffer_.size() == 1)
      { // Looking for SYNC1
        if (*data_ == novatel::SYNC_1)
        {
          buffer_.push_back(*data_++);
        }
        else
        {
          buffer_.clear();
        }
      }
      else if (buffer_.size() == 2)
      { // Looking for SYNC2
        switch (*data_)
        {
        case novatel::SYNC_2_LONG_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          buffer_.clear();
        }
      }
      else if (header_length_ > 0)
      { // Working on header.
        if (buffer_.size() < header_length_)
        {
          buffer_.push_back(*data_++);
        }
        else
        {
          if (header_length_ == sizeof(novatel::LongHeader))
          {
            total_length_ = header_length_ + novatel::CRC_LENGTH +
                            reinterpret_cast<novatel::LongHeader *>(buffer_.data())->message_length;
          }
          else if (header_length_ == sizeof(novatel::ShortHeader))
          {
            total_length_ = header_length_ + novatel::CRC_LENGTH +
                            reinterpret_cast<novatel::ShortHeader *>(buffer_.data())->message_length;
          }
          else
          {
            std::cout << "Incorrect header_length_. Should never reach here.\n";
            buffer_.clear();
          }

          header_length_ = 0;
        }
      }
      else if (total_length_ > 0)
      {
        if (buffer_.size() < total_length_)
        { // Working on body.
          buffer_.push_back(*data_++);
          continue;
        }

        MessageType type = PrepareMessage(message_ptr);
        buffer_.clear();
        total_length_ = 0;
        if (type != MessageType::NONE)
        {
          return type;
        }
      }
    }

    return MessageType::NONE;
  }

  MessageType NovatelParser::PrepareMessage(MessagePtr *message_ptr)
  {
    if (!check_crc())
    {
      std::cout << "CRC check failed.\n";
      return MessageType::NONE;
    }

    uint8_t *message = nullptr;
    novatel::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;
    if (buffer_[2] == novatel::SYNC_2_LONG_HEADER)
    {
      auto header = reinterpret_cast<const novatel::LongHeader *>(buffer_.data());
      message = buffer_.data() + sizeof(novatel::LongHeader);
      gps_week = header->gps_week;
      gps_millisecs = header->gps_millisecs;
      message_id = header->message_id;
      message_length = header->message_length;
    }
    else
    {
      auto header = reinterpret_cast<const novatel::ShortHeader *>(buffer_.data());
      message = buffer_.data() + sizeof(novatel::ShortHeader);
      gps_week = header->gps_week;
      gps_millisecs = header->gps_millisecs;
      message_id = header->message_id;
      message_length = header->message_length;
    }

    switch (message_id)
    {
    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
    case novatel::IMURATECORRIMUS:
      if (message_length != sizeof(novatel::CorrImuData))
      {
        std::cout << "Incorrect message_length\n";
        break;
      }
      has_corr_imu_message_ = true;
      if (HandleCorrImuData(reinterpret_cast<novatel::CorrImuData *>(message)))
      {
        *message_ptr = &corrimu_;
        return MessageType::CORRIMUDATAS;
      }
      break;

    case novatel::INSPVAX:
      if (message_length != sizeof(novatel::InsPvaX))
      {
        std::cout << "Incorrect message_length\n";
        break;
      }
      has_ins_stat_message_ = true;
      if (HandleInsPvax(reinterpret_cast<novatel::InsPvaX *>(message), gps_week, gps_millisecs))
      {
        *message_ptr = &inspvax_;
        return MessageType::INSPVAX;
      }
      break;
    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
      if (message_length != sizeof(novatel::RawImuX))
      {
        std::cout << "Incorrect message_length\n";
        break;
      }

      if (HandleRawImuX(reinterpret_cast<novatel::RawImuX *>(message)))
      {
        *message_ptr = &rawimu_;
        return MessageType::RAWIMUX;
      }
      break;

    default:
      break;
    }

    return MessageType::NONE;
  }

  bool NovatelParser::check_crc()
  {
    size_t l = buffer_.size() - novatel::CRC_LENGTH;
    return CalculateBlockCRC32(l, (unsigned char *)buffer_.data()) == *(uint32_t *)(buffer_.data() + l);
  }

  bool NovatelParser::HandleInsPvax(const novatel::InsPvaX *pvax, uint16_t gps_week, uint32_t gps_millisecs)
  {
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    double unix_sec = gnss::util::gps2unix(seconds);

    // std::cout << std::setprecision(10);
    // std::cout << "unix_sec: " << unix_sec << "\n";
    // std::cout << "ros_time: " << rclcpp::Clock().now().seconds() << "\n";

    inspvax_.header.stamp = rclcpp::Time(static_cast<int64_t>(unix_sec * 1e9));
    inspvax_.latitude = pvax->latitude;
    inspvax_.longitude = pvax->longitude;
    inspvax_.height = pvax->height;
    inspvax_.undulation = pvax->undulation;
    inspvax_.north_velocity = pvax->north_velocity;
    inspvax_.east_velocity = pvax->east_velocity;
    inspvax_.up_velocity = pvax->up_velocity;
    inspvax_.roll = pvax->roll;
    inspvax_.pitch = pvax->pitch;
    inspvax_.azimuth = pvax->azimuth;
    inspvax_.latitude_std = pvax->latitude_std;
    inspvax_.longitude_std = pvax->longitude_std;
    inspvax_.height_std = pvax->height_std;
    inspvax_.north_velocity_std = pvax->north_velocity_std;
    inspvax_.east_velocity_std = pvax->east_velocity_std;
    inspvax_.up_velocity_std = pvax->up_velocity;
    inspvax_.roll_std = pvax->roll_std;
    inspvax_.pitch_std = pvax->pitch_std;
    inspvax_.azimuth_std = pvax->azimuth_std;

    return true;
  }

  bool NovatelParser::HandleCorrImuData(const novatel::CorrImuData *imu)
  {
    double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    double unix_sec = gnss::util::gps2unix(seconds);

    // x, y, z
    corrimu_.header.stamp = rclcpp::Time(static_cast<int64_t>(unix_sec * 1e9));
    corrimu_.x_linear_acceleration = imu->x_velocity_change * corrimudata_hz_;
    corrimu_.y_linear_acceleration = imu->y_velocity_change * corrimudata_hz_;
    corrimu_.z_linear_acceleration = imu->z_velocity_change * corrimudata_hz_;
    corrimu_.x_angular_velocity = imu->x_angle_change * corrimudata_hz_;
    corrimu_.y_angular_velocity = imu->y_angle_change * corrimudata_hz_;
    corrimu_.z_angular_velocity = imu->y_angle_change * corrimudata_hz_;

    return true;
  }

  bool NovatelParser::HandleRawImuX(const novatel::RawImuX *imu)
  {
    if (imu->imu_error != 0)
    {
      std::cout << "IMU error. Status: " << std::hex << std::showbase
                << imu->imuStatus << "\n";
    }

    if (is_zero(gyro_scale_))
    {
      novatel::ImuType imu_type = imu_type_;
      novatel::ImuParameter param = novatel::GetImuParameter(imu_type);
      std::cout << "IMU type: " << "G320N" << "; "
                << "Gyro scale: " << param.gyro_scale << "; "
                << "Accel scale: " << param.accel_scale << "; "
                << "Sampling rate: " << param.sampling_rate_hz << ".\n";

      if (is_zero(param.sampling_rate_hz))
      {
        std::cout << "Unsupported IMU type: "
                  << "Unknown..\n";
        return false;
      }

      gyro_scale_ = param.gyro_scale * param.sampling_rate_hz;
      accel_scale_ = param.accel_scale * param.sampling_rate_hz;
      imu_measurement_hz_ = static_cast<float>(param.sampling_rate_hz);
      imu_measurement_span_ = static_cast<float>(1.0 / param.sampling_rate_hz);
    }

    double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    double unix_sec = gnss::util::gps2unix(time);
    if (imu_measurement_time_previous_ > 0.0 && fabs(time - imu_measurement_time_previous_ - imu_measurement_span_) > 1e-4)
    {
      std::cout << "Unexpected delay between two IMU measurements at: "
                << time - imu_measurement_time_previous_
                << "\n";
    }

    // x y z
    rawimu_.header.stamp = rclcpp::Time(static_cast<int64_t>(unix_sec * 1e9));
    rawimu_.x_linear_acceleration = -imu->x_velocity_change * accel_scale_;
    rawimu_.y_linear_acceleration = -imu->y_velocity_change_neg * accel_scale_;
    rawimu_.z_linear_acceleration = -imu->z_velocity_change * accel_scale_;
    rawimu_.x_angular_velocity = -imu->x_angle_change * gyro_scale_;
    rawimu_.y_angular_velocity = -imu->y_angle_change_neg * gyro_scale_;
    rawimu_.z_angular_velocity = -imu->z_angle_change * gyro_scale_;

    imu_measurement_time_previous_ = time;
    return true;
  }

} // namespace gnss
