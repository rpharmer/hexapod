#include "visualiser/robot/enum_names.hpp"

namespace visualiser::robot {

const char* RobotModeName(int mode) {
  switch (mode) {
    case 0:
      return "SAFE_IDLE";
    case 1:
      return "HOMING";
    case 2:
      return "STAND";
    case 3:
      return "WALK";
    case 4:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

const char* FaultCodeName(int fault) {
  switch (fault) {
    case 0:
      return "NONE";
    case 1:
      return "BUS_TIMEOUT";
    case 2:
      return "ESTOP";
    case 3:
      return "TIP_OVER";
    case 4:
      return "ESTIMATOR_INVALID";
    case 5:
      return "MOTOR_FAULT";
    case 6:
      return "JOINT_LIMIT";
    case 7:
      return "COMMAND_TIMEOUT";
    default:
      return "UNKNOWN";
  }
}

const char* NavigationLifecycleName(int value) {
  switch (value) {
    case 0:
      return "Idle";
    case 1:
      return "Running";
    case 2:
      return "Paused";
    case 3:
      return "Blocked";
    case 4:
      return "MapUnavailable";
    case 5:
      return "Completed";
    case 6:
      return "Failed";
    case 7:
      return "Cancelled";
    default:
      return "Unknown";
  }
}

const char* LocalPlanStatusName(int value) {
  switch (value) {
    case 0:
      return "Ready";
    case 1:
      return "GoalReached";
    case 2:
      return "Blocked";
    case 3:
      return "MapUnavailable";
    default:
      return "Unknown";
  }
}

const char* PlannerBlockReasonName(int value) {
  switch (value) {
    case 0:
      return "None";
    case 1:
      return "StartOccupied";
    case 2:
      return "GoalOccupied";
    case 3:
      return "NoPath";
    case 4:
      return "SearchBudgetExceeded";
    default:
      return "Unknown";
  }
}

}  // namespace visualiser::robot
