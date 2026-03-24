#include "runtime_teardown.hpp"

#include <exception>

#include "shutdown_summary.hpp"

namespace app {

bool runRuntimeTeardown(const std::shared_ptr<logging::AsyncLogger>& logger,
                        const std::function<void()>& stop_fn,
                        int runner_rc)
{
  bool teardown_ok = true;
  try {
    stop_fn();
  } catch (const std::exception& ex) {
    teardown_ok = false;
    LOG_ERROR(logger, "Robot teardown failed: ", ex.what());
  } catch (...) {
    teardown_ok = false;
    LOG_ERROR(logger, "Robot teardown failed: unknown exception");
  }

  if (teardown_ok) {
    LOG_INFO(logger, "All workers joined");
  } else {
    LOG_ERROR(logger, "All workers joined: false");
  }

  const std::size_t dropped_messages = logger->DroppedMessageCount();
  if (dropped_messages > 0) {
    LOG_ERROR(logger, "Dropped messages=", dropped_messages);
  } else {
    LOG_INFO(logger, "Dropped messages=", dropped_messages);
  }

  logShutdownSummary(logger, teardown_ok, dropped_messages, runner_rc);
  return teardown_ok;
}

} // namespace app
