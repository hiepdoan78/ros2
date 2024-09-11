#ifndef CARLIKEBOT_CONFIG_HPP
#define CARLIKEBOT_CONFIG_HPP

#include <string>

struct Config
{
  std::string device = "/dev/ttyAMA10";
  int baud_rate = 460800;
  int timeout = 200; // Timeout in milliseconds
};

#endif // CARLIKEBOT_CONFIG_HPP
