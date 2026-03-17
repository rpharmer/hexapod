#pragma once

#include "types.hpp"

class LegFK {
public:
  LegFK();
  LegTargets solve(const RawHardwareState& raw, const SafetyState& safety);
  FootTarget footInBodyFrame(const LegRawState& est, const LegGeometry& leg);
  FootTarget footInWorldFrame(const LegRawState& est, const BodyPose& bodyPose,
                             const LegGeometry& leg);

private:
  bool solveOneLeg(const LegRawState& est, FootTarget& out,
                   const LegGeometry& leg);
  uint64_t seq_tx_{0};

  HexapodGeometry hexGeo;
};