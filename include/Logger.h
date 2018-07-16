#ifndef SLAM_LOGGER_H
#define SLAM_LOGGER_H

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#define CONSOLE console

namespace SLAM{
  
  auto console = spdlog::stdout_color_mt("console");
  
}

#endif