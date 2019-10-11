#pragma once

#include <iostream>

// SPD Logger
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"


namespace mfuse
{
class Logger 
{
private:

  int logloopTimeoutMilliseconds_ = 250;
  std::shared_ptr<spdlog::logger> logger_;
  std::thread log_thread; 
  int logloop();

public:

  explicit Logger();
  ~Logger();

  std::shared_ptr<spdlog::logger> CreateLogger(std::string logDirectory, std::string logName);

};
} // namespace mfuse