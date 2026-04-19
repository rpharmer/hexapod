#pragma once

#include "types.hpp"

namespace contact_foot_response {

/** True when foot contact bits and body twist from the estimator are trusted for control mods. */
bool sensorsTrustedForContactResponse(const RobotState& est);

/**
 * During swing: if foot reports contact, snap swing progress to touchdown (tau=1).
 * If near touchdown and still airborne, add negative Z in the planning frame (reach down).
 */
void adjustSwingTauAndVerticalExtension(bool in_swing,
                                        bool foot_contact,
                                        const RobotState& est,
                                        double tau01,
                                        double& tau_out,
                                        double& extra_down_z_m,
                                        const FootContactFusion* contact_fusion = nullptr);

/**
 * Stance / stand: small vertical shift per foot from roll/pitch estimate vs intent (body planning frame).
 */
double stanceTiltLevelingDeltaZ(const RobotState& est,
                                const MotionIntent& intent,
                                double anchor_x,
                                double anchor_y);

} // namespace contact_foot_response
