#pragma once

#include "types.hpp"

class LegFK {
public:
  LegFK();
  /** Input leg angles are in calibrated servo space, matching RobotState/JointTargets. */
  LegTargets solve(const RobotState& raw, const SafetyState& safety);
  FootTarget footInBodyFrame(const LegState& servo_state, const LegGeometry& leg);
  FootTarget footInWorldFrame(const LegState& servo_state, const BodyPose& bodyPose,
                             const LegGeometry& leg);

private:
  bool solveOneLeg(const LegState& servo_state, FootTarget& out,
                   const LegGeometry& leg);
  uint64_t seq_tx_{0};

  HexapodGeometry hexGeo;
};
