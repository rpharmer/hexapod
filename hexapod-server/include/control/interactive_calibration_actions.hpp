#ifndef INTERACTIVE_CALIBRATION_ACTIONS_HPP
#define INTERACTIVE_CALIBRATION_ACTIONS_HPP

#include <memory>

#include "interactive_input_mapper.hpp"
#include "logger.hpp"

bool executeCalibrationAction(CalibrationAction action,
                              const std::shared_ptr<logging::AsyncLogger>& logger);

#endif // INTERACTIVE_CALIBRATION_ACTIONS_HPP
