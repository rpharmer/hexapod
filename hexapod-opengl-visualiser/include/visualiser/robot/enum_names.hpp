#pragma once

#include <optional>
#include <string_view>

namespace visualiser::robot {

const char* RobotModeName(int mode);
const char* FaultCodeName(int fault);
std::optional<int> ParseFaultCodeName(std::string_view name);
const char* NavigationLifecycleName(int value);
const char* LocalPlanStatusName(int value);
const char* PlannerBlockReasonName(int value);

}  // namespace visualiser::robot
