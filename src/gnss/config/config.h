#pragma once

namespace config {

class Config
{
public:
  // static Config& GetInstance()
  // {
  //   static Config config;
  //   return config;
  // }

  const char* device = "/dev/ttyUSB0";
  uint32_t baud_rate = 460800;

private:
  // Config();
  // Config(const Config& config) = delete;
  // const Config& operator=(const Config& config) = delete;
};

} // namespace config

