#include <mfuse_logger.hpp>

namespace mfuse
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Logger::Logger()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Logger::~Logger()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int Logger::logloop()
{
  while(true)
  {
    if(NULL != logger_)
      logger_->flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(logloopTimeoutMilliseconds_));
  }
}

//*****************************************************************************
//*
//* decsription: Create two log sinks.
//*   console : warnings and above
//*   file : trace (everything) and above
//* info: https://github.com/gabime/spdlog
//* examples:
//*   logger.set_level(spdlog::level::debug);
//*   spdlog::info("Welcome to spdlog!");
//*   spdlog::error("Some error message with arg: {}", 1);
//*   spdlog::warn("Easy padding in numbers like {:08d}", 12);
//*   spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
//*   spdlog::info("Support for floats {:03.2f}", 1.23456);
//*   spdlog::info("Positional args are {1} {0}..", "too", "supported");
//*   spdlog::info("{:<30}", "left aligned");
//* Change log pattern
//*   spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
//* Compile time log levels, define SPDLOG_ACTIVE_LEVEL to desired level
//*   SPDLOG_TRACE("Some trace message with param {}", {});
//*   SPDLOG_DEBUG("Some debug message");
//*
//******************************************************************************

std::shared_ptr<spdlog::logger> Logger::CreateLogger(std::string logDirectory, std::string logName)
{
    spdlog::info("Creating logs");

    try 
    {
      time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];
      spdlog::level::level_enum logLevel_ = spdlog::level::debug;
      std::string logLevelName = "---";

      // we need to create a log flush thread, because the one built in to spdlog
      // doesn't work
      log_thread = std::thread(&Logger::logloop, this); 

      // create time string
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
      auto fileName = logDirectory + logName + "_Log_" + std::string(buffer) + ".txt";

      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");
      
      // create file logger
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(fileName, true);
      file_sink->set_level(spdlog::level::trace);
      file_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      // create multi logger
      spdlog::sinks_init_list sl = {console_sink, file_sink};
      logger_ = std::make_shared<spdlog::logger>(logName, sl);

      switch(logLevel_)
      {
        case spdlog::level::off : logLevelName = "off"; break;
        case spdlog::level::trace : logLevelName = "trace"; break;
        case spdlog::level::debug : logLevelName = "debug"; break;
        case spdlog::level::info : logLevelName = "info"; break;
        case spdlog::level::warn : logLevelName = "warn"; break;
        case spdlog::level::err : logLevelName = "err"; break;
        case spdlog::level::critical : logLevelName = "critical"; break;
      }

      // set level low to show info on startup
      logger_->set_level(spdlog::level::trace);
      spdlog::info("log file created: {0}, level: {1}", fileName, logLevelName);

      // set level to normal going forward
      logger_->set_level(logLevel_);
      logger_->warn("this should appear in both console and file");
      logger_->info("this message should not appear in the console, only in the file");

      // these dont seem to work, but maybe someday they will
      logger_->flush_on(spdlog::level::err);
      spdlog::flush_on(spdlog::level::err);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
      std::cout << "Log init failed: " << ex.what() << std::endl;
            
      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      spdlog::sinks_init_list sl = {console_sink};
      logger_ = std::make_shared<spdlog::logger>(logName, sl);
	}   

    return logger_;  
}
} // namespace mfuse