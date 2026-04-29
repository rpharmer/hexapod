#pragma once

namespace visualiser::robot {

const char* RobotModeName(int mode);
const char* FaultCodeName(int fault);
const char* NavigationLifecycleName(int value);
const char* LocalPlanStatusName(int value);
const char* PlannerBlockReasonName(int value);

}  // namespace visualiser::robot
