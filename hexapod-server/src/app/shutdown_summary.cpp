#include "shutdown_summary.hpp"

namespace app {

void logShutdownSummary(const std::shared_ptr<logging::AsyncLogger>& logger,
                        bool teardown_ok,
                        std::size_t dropped_messages,
                        int runner_rc)
{
  const bool shutdown_success = teardown_ok && dropped_messages == 0;
  const logging::LogLevel shutdown_summary_level =
      shutdown_success ? logging::LogLevel::Info : logging::LogLevel::Error;

  logger->LogStream(shutdown_summary_level,
                    LOG_SOURCE_LOCATION,
                    "shutdown_summary"
                    " success=",
                    shutdown_success ? 1 : 0,
                    " teardown_ok=",
                    teardown_ok ? 1 : 0,
                    " dropped_messages=",
                    dropped_messages,
                    " runner_rc=",
                    runner_rc);
}

} // namespace app
