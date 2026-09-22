#include "minphys3d/demo/pinocchio_hexapod.hpp"
#include "hexapod_dynamics_constants.hpp"
#include "control/joint_angle_gravity_feedforward.hpp"
#include "hardware/physics_sim_joint_wire_mapping.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace minphys3d::demo {
namespace {
// Restore physical poses bit-for-bit after geometric queries: a q/v round
// trip alone introduces rounding noise into the free-chassis contact history.
struct PoseRestore {
    World& world;
    struct Entry { std::uint32_t id; Vec3 p, v, w; Quat q; bool sleeping; };
    std::vector<Entry> entries;
    PoseRestore(World& w, const HexapodSceneObjects& s) : world(w) {
        auto save = [&](std::uint32_t id) {
            const auto& b = w.GetBody(id);
            entries.push_back({id, b.position, b.velocity, b.angularVelocity, b.orientation, b.isSleeping});
        };
        save(s.body);
        for (const auto& leg : s.legs) { save(leg.coxa); save(leg.femur); save(leg.tibia); }
    }
    ~PoseRestore() {
        for (const auto& e : entries) {
            auto& b = world.GetBody(e.id);
            b.position=e.p; b.velocity=e.v; b.angularVelocity=e.w; b.orientation=e.q; b.isSleeping=e.sleeping;
        }
    }
};
Vec3 foot(const World& w, const HexapodSceneObjects& s, int leg) {
    const auto& b = w.GetBody(s.legs[leg].tibia);
    for (const auto& c : b.compoundChildren)
        if (c.shape == ShapeType::Sphere) return b.position + Rotate(b.orientation, c.localPosition);
    throw std::runtime_error("missing sphere");
}
double radius(const World& w, const HexapodSceneObjects& s, int leg) {
    for (const auto& c : w.GetBody(s.legs[leg].tibia).compoundChildren)
        if (c.shape == ShapeType::Sphere) return c.radius;
    throw std::runtime_error("missing sphere");
}
double potential(const World& w, const HexapodSceneObjects& s) {
    auto bodyEnergy = [&](std::uint32_t id) {
        const auto& b = w.GetBody(id);
        return -b.mass * Dot(w.GetGravity(), b.position);
    };
    double result = bodyEnergy(s.body);
    for (const auto& leg : s.legs)
        result += bodyEnergy(leg.coxa) + bodyEnergy(leg.femur) + bodyEnergy(leg.tibia);
    return result;
}
// Independent gravity oracle from physical-body potential, not Pinocchio RNEA.
std::array<double, 3> gravityGradient(PinocchioHexapodModel& model, World& w,
        const HexapodSceneObjects& scene, int leg, const std::vector<double>& q) {
    PoseRestore exactRestore(w, scene);
    std::vector<double> restoreQ, restoreV;
    model.readState(w, restoreQ, restoreV);
    const std::vector<double> zero(restoreV.size(), 0);
    std::array<double, 3> result{};
    for (int j = 0; j < 3; ++j) {
        auto perturbed = q;
        perturbed[7 + 3*leg + j] += 1e-6;
        model.writeState(w, perturbed, zero);
        const double plus = potential(w, scene);
        perturbed[7 + 3*leg + j] -= 2e-6;
        model.writeState(w, perturbed, zero);
        result[j] = (plus - potential(w, scene)) / 2e-6;
    }
    model.writeState(w, restoreQ, restoreV);
    return result;
}
struct Balance {
    int samples = 0;
    double error{}, velocity{}, acceleration{}, kp{}, kd{}, requested{}, applied{}, gravity{}, gravityFD{};
    double bias{}, inertial{}, contact{}, predictedError{};
    double maxMotorResidual{}, maxGravityResidual{}, maxContact{}, maxVelocity{}, maxAcceleration{};
};
struct MovingBalance {
    int samples{};
    double errorAbs{}, gravityAbs{}, dampingAbs{}, inertiaAbs{}, coriolisAbs{}, contactAbs{}, saturationAbs{}, biasAbs{};
    double maxClosure{}, maxMotorResidual{};
};
// Test-local geometric IK of the actual compound sphere. Every invocation
// restores the physical state; no ideal joint motion is used in the simulation.
bool ik(PinocchioHexapodModel& model, World& w, const HexapodSceneObjects& scene,
        int leg, const Vec3& target, std::vector<double>& desired) {
    PoseRestore exactRestore(w, scene);
    std::vector<double> actual, velocity;
    model.readState(w, actual, velocity);
    const std::vector<double> zero(velocity.size(), 0);
    bool ok = false;
    for (int iteration = 0; iteration < 30; ++iteration) {
        model.writeState(w, desired, zero);
        const Vec3 p = foot(w, scene, leg), error = target - p;
        if (Length(error) < 1e-8) { ok = true; break; }
        Mat3 J{};
        for (int j = 0; j < 3; ++j) {
            auto perturbed = desired;
            perturbed[7 + 3*leg + j] += 1e-6;
            model.writeState(w, perturbed, zero);
            const Vec3 d = (foot(w, scene, leg) - p) / 1e-6;
            J.m[0][j] = d.x; J.m[1][j] = d.y; J.m[2][j] = d.z;
        }
        Mat3 inv{};
        if (!InvertMat3(J, inv)) break;
        const Vec3 delta = inv * error;
        const std::array<double, 3> a{delta.x, delta.y, delta.z};
        for (int j = 0; j < 3; ++j)
            desired[7 + 3*leg + j] += std::clamp(a[j], -.1, .1);
    }
    model.writeState(w, actual, velocity);
    return ok;
}
bool run(bool pedestal, int lifted, bool gravityBias, double biasStart, bool serverBias, bool reportedStiffness,
         bool movingAudit = false, bool velocityLead = false) {
    World world({0, -9.80665, 0});
    auto scene = BuildHexapodScene(world);
    std::array<std::uint32_t, 18> ids{};
    for (int l = 0; l < 6; ++l) {
        ids[3*l] = scene.legs[l].bodyToCoxaJoint;
        ids[3*l+1] = scene.legs[l].coxaToFemurJoint;
        ids[3*l+2] = scene.legs[l].femurToTibiaJoint;
    }
    PinocchioHexapodModel model(world, scene, ids);
    ::LegGeometry serverLeg{};
    const auto initialAxis = Rotate(world.GetBody(scene.legs[lifted].coxa).orientation, Vec3{1,0,0});
    serverLeg.mountAngle = AngleRad{::kPi-std::atan2(initialAxis.x,-initialAxis.z)};
    serverLeg.coxaLength=LengthM{.043}; serverLeg.femurLength=LengthM{.060}; serverLeg.tibiaLength=LengthM{.104};
    model.setCommandInterval(.005);
    std::vector<double> q, v;
    model.readState(world, q, v);
    std::array<double, 18> zeros{};
    for (int j = 0; j < 18; ++j) zeros[j] = world.GetServoJointAngle(ids[j]) - q[7+j];
    std::array<Vec3, 6> points{};
    for (int l = 0; l < 6; ++l) { points[l] = foot(world, scene, l); points[l].y = radius(world, scene, l); }
    q[1] = .14;
    for (int l = 0; l < 6; ++l)
        if (!ik(model, world, scene, l, points[l], q)) throw std::runtime_error("stand IK failed");
    if (!model.writeState(world, q, v)) throw std::runtime_error("stand write failed");
    // Verify the assumed wire/config ordering by reading every physical hinge.
    for (int j = 0; j < 18; ++j) {
        if (std::abs(std::remainder(world.GetServoJointAngle(ids[j]) - zeros[j] - q[7+j], 6.28318530718)) > 1e-7)
            throw std::runtime_error("wire/config map mismatch");
        world.GetServoJointMutable(ids[j]).targetAngle = zeros[j] + q[7+j];
    }
    if (pedestal) {
        Body support;
        support.shape = ShapeType::Box;
        support.isStatic = true;
        support.halfExtents = {.05, .03, .05};
        support.position = {q[0], world.GetBody(scene.body).ComputeAABB().min.y - .03, q[2]};
        support.restitution = 0;
        support.staticFriction = support.dynamicFriction = 2;
        support.collisionGroup = 0x0004;
        support.collisionMask = 0x0002; // chassis only, never the feet
        world.CreateBody(support);
    }
    ProximalSolverSettings settings;
    settings.maxIterations = 24;
    constexpr double dt = 1.0/600;
    int held = 0, recovered = 0, contacts = 0;
    double startY = 0, peakLift = -1e9, firstLoss = -1, peakW = 0, peakTorque = 0;
    double minBody = 1e9, maxBody = -1e9, maxTracking = 0;
    auto desired = q;
    const auto referenceBody = world.GetBody(scene.body);
    Vec3 entryBody{}, entryLocal{};
    Quat entryOrientation{};
    double initialCommandGap = 0;
    std::array<Balance, 3> balance{};
    std::array<std::array<MovingBalance, 3>, 2> movingBalance{}; // unloaded, loaded
    const double rampS = movingAudit ? .25 : 1.0;
    int movingExcluded = 0, movingSamples = 0;
    double movingErrorSq = 0, apexTrackingError = 0, maxReferenceRate = 0, maxBias = 0;
    Vec3 commandPoint = points[lifted];
    int balanceExcluded = 0, balanceContacts = 0;
    std::array<double, 3> targetBias{};
    std::array<double, 3> serverStiffness{};
    bool previousLiftContact = true;
    int biasedContactSamples = 0;
    for (int tick = 0; tick < 3600; ++tick) {
        const double t = tick * dt - 3.0;
        if (tick % 3 == 0 && t >= 0) {
            const double u = std::clamp(t / rampS, 0.0, 1.0);
            const double smooth = u*u*u*(10 + u*(-15 + 6*u));
            Vec3 target = points[lifted]; target.y += .03*smooth;
            commandPoint = target;
            const auto previousDesired = desired;
            if (!ik(model, world, scene, lifted, target, desired)) throw std::runtime_error("lift IK failed");
            for (int j=0;j<3;++j)
                maxReferenceRate = std::max(maxReferenceRate, std::abs((desired[7+3*lifted+j]-previousDesired[7+3*lifted+j])/.005));
            for (int j = 0; j < 18; ++j) world.GetServoJointMutable(ids[j]).targetAngle = zeros[j] + desired[7+j];
            // Test-only, selected leg only. Default experiment starts after
            // liftoff; the separate onset experiment includes load transfer.
            // g/Kp is a target offset equivalent to adding gravity torque in
            // this unsaturated PD law; no new force or chassis actuation.
            if (gravityBias && t >= biasStart) {
                if (biasStart > 0 && previousLiftContact)
                    throw std::runtime_error("late gravity-bias experiment requires unloaded leg");
                PinocchioHexapodModel::ServoBalanceSample last;
                if (!model.debugLastServoBalance(last)) throw std::runtime_error("missing gain for gravity bias");
                std::vector<double> liveQ, liveV;
                model.readState(world, liveQ, liveV);
                const auto g = gravityGradient(model, world, scene, lifted, liveQ);
                ::LegGravityCompensation serverComp{};
                if (serverBias) {
                    control_config::GravityFeedforwardConfig cfg{};
                    cfg.include_self_weight=true; cfg.include_foot_reaction=false;
                    cfg.scale_femur=cfg.scale_tibia=1;
                    const auto down = Rotate(Conjugate(world.GetBody(scene.body).orientation), Vec3{0,-1,0});
                    std::array<double,3> mechanical{};
                    std::array<double,3> actuatorStiffness{};
                    const auto reported = model.servoStiffnessNmPerRad();
                    for (int j=0;j<3;++j) actuatorStiffness[j] = reported[3*lifted+j];
                    for (int j=0;j<3;++j) mechanical[j] = physics_sim_joint_wire_mapping::jointMechanicalFromSimWireAngle(
                        j, world.GetServoJointAngle(ids[3*lifted+j]));
                    serverComp=computeLegGravityCompensation(serverLeg,mechanical[0],mechanical[1],mechanical[2],
                                                            ::Vec3{-down.z,down.x,down.y},0,cfg,
                                                            reportedStiffness ? &actuatorStiffness : nullptr);
                    serverStiffness={0,serverComp.stiffness_femur_nm_per_rad,serverComp.stiffness_tibia_nm_per_rad};
                }
                const double blendU = std::clamp((t-biasStart)/.25, 0.0, 1.0);
                const double blend = blendU*blendU*blendU*(10+blendU*(-15+6*blendU));
                for (int j = 0; j < 3; ++j) {
                    const int wire = 3*lifted+j;
                    const double wn = hexapod_dynamics::kServoOmegaN;
                    const double kp = movingAudit ? model.servoStiffnessNmPerRad()[wire]
                        : last.gainScale*last.effectiveInertias[wire]*wn*wn;
                    targetBias[j] = blend*g[j]/kp;
                    if (serverBias) targetBias[j] = blend * (j==0 ? serverComp.delta_coxa_rad
                        : j==1 ? serverComp.delta_femur_rad : serverComp.delta_tibia_rad);
                    const double referenceRate = (desired[7+wire] - previousDesired[7+wire]) / .005;
                    maxReferenceRate = std::max(maxReferenceRate, std::abs(referenceRate));
                    // Test-only PD reference-velocity compensation. No gain or
                    // torque-envelope change: Kp * lead = Kd * qdot_reference.
                    if (velocityLead) targetBias[j] += 2*hexapod_dynamics::kServoZeta/wn * referenceRate;
                    maxBias = std::max(maxBias, std::abs(targetBias[j]));
                    world.GetServoJointMutable(ids[wire]).targetAngle += targetBias[j];
                }
            }
        }
        if (tick == 1800) {
            const auto& b = world.GetBody(scene.body);
            entryBody = b.position;
            entryOrientation = b.orientation;
            entryLocal = Rotate(Conjugate(b.orientation), foot(world, scene, lifted) - b.position);
            startY = foot(world, scene, lifted).y;
            const Vec3 commandLocal = Rotate(Conjugate(referenceBody.orientation), points[lifted] - referenceBody.position);
            initialCommandGap = (b.position + Rotate(b.orientation, commandLocal)).y - startY;
        }
        const bool audit = t >= 2.5 && tick % 3 == 0;
        const bool movingSample = movingAudit && t >= 0 && t <= rampS + .15 && tick % 3 == 0;
        std::vector<double> beforeQ, beforeV;
        if (audit || movingSample) model.readState(world, beforeQ, beforeV);
        ProximalStepDiagnostics d;
        if (!model.stepProximal(world, dt, settings, d)) ++held;
        recovered += d.status == ProximalStepStatus::RecoveredRetry;
        previousLiftContact = d.legContactCount[lifted] > 0;
        biasedContactSamples += gravityBias && t >= biasStart && previousLiftContact;
        if ((audit || movingSample) && d.status == ProximalStepStatus::Healthy) {
            PinocchioHexapodModel::ServoBalanceSample actual;
            if (!model.debugLastServoBalance(actual) || std::abs(actual.subDt-dt) > 1e-12)
                throw std::runtime_error("missing full-step actuator sample");
            for (int wire = 0; wire < 18; ++wire) {
                const double kp = actual.gainScale * actual.effectiveInertias[wire]
                    * hexapod_dynamics::kServoOmegaNSq();
                if (std::abs(model.servoStiffnessNmPerRad()[wire] - kp) > 1e-10)
                    throw std::runtime_error("reported stiffness differs from actual healthy PD gain");
            }
            std::vector<double> afterQ, afterV;
            model.readState(world, afterQ, afterV);
            PinocchioHexapodModel::ImplicitDampingOracleResult moving, stationary;
            if (!model.computeImplicitDampingOracle(beforeQ, beforeV, actual.errors,
                    actual.effectiveInertias, dt, moving, actual.gainScale)
                || !model.computeImplicitDampingOracle(beforeQ, std::vector<double>(beforeV.size(), 0),
                    actual.errors, actual.effectiveInertias, dt, stationary, actual.gainScale))
                throw std::runtime_error("balance mass/bias oracle failed");
            const auto gravityFD = gravityGradient(model, world, scene, lifted, beforeQ);
            if (audit) balanceContacts += d.legContactCount[lifted] > 0;
            for (int j = 0; j < 3; ++j) {
                auto& b = balance[j];
                const int wire = 3*lifted+j, vi = 6+wire;
                const double wn = hexapod_dynamics::kServoOmegaN;
                const double kp = actual.gainScale * actual.effectiveInertias[wire] * wn*wn;
                const double kd = actual.dampingGainScale * actual.effectiveInertias[wire]
                    * 2*hexapod_dynamics::kServoZeta*wn;
                const double requested = kp*actual.errors[wire] - kd*beforeV[vi];
                double available = hexapod_dynamics::kServoMaxTorqueNm;
                if (requested*beforeV[vi] > 0)
                    available *= std::max(0.0, 1-std::abs(beforeV[vi])/hexapod_dynamics::kServoNoLoadSpeedRadPerSec);
                const double expectedTorque = std::clamp(requested, -available, available);
                double inertiaTerm = 0;
                for (std::size_t col = 0; col < beforeV.size(); ++col)
                    inertiaTerm += moving.mass[vi + col*beforeV.size()] * (afterV[col]-beforeV[col])/dt;
                const double contact = inertiaTerm + moving.bias[vi] - actual.appliedTorques[wire];
                const double acceleration = (afterV[vi]-beforeV[vi])/dt;
                if (movingSample) {
                    auto& mb = movingBalance[d.legContactCount[lifted] > 0 ? 1 : 0][j];
                    ++mb.samples;
                    const double gravityError = stationary.bias[vi]/kp;
                    const double dampingError = kd*beforeV[vi]/kp;
                    const double inertiaError = inertiaTerm/kp;
                    const double coriolisError = (moving.bias[vi]-stationary.bias[vi])/kp;
                    const double contactError = -contact/kp;
                    const double saturationError = (requested-actual.appliedTorques[wire])/kp;
                    const double referenceError = actual.errors[wire]-targetBias[j];
                    const double predicted = gravityError+dampingError+inertiaError+coriolisError
                        +contactError+saturationError-targetBias[j];
                    mb.errorAbs += std::abs(referenceError);
                    mb.gravityAbs += std::abs(gravityError); mb.dampingAbs += std::abs(dampingError);
                    mb.inertiaAbs += std::abs(inertiaError); mb.coriolisAbs += std::abs(coriolisError);
                    mb.contactAbs += std::abs(contactError); mb.saturationAbs += std::abs(saturationError);
                    mb.biasAbs += std::abs(targetBias[j]);
                    mb.maxClosure = std::max(mb.maxClosure, std::abs(referenceError-predicted));
                    mb.maxMotorResidual = std::max(mb.maxMotorResidual, std::abs(actual.appliedTorques[wire]-expectedTorque));
                }
                if (!audit) continue;
                ++b.samples;
                b.error += actual.errors[wire]; b.velocity += beforeV[vi]; b.acceleration += acceleration;
                b.kp += kp; b.kd += kd; b.requested += requested; b.applied += actual.appliedTorques[wire];
                b.gravity += stationary.bias[vi]; b.gravityFD += gravityFD[j];
                b.bias += moving.bias[vi]; b.inertial += inertiaTerm; b.contact += contact;
                b.predictedError += (stationary.bias[vi] + kd*beforeV[vi])/kp;
                b.maxMotorResidual = std::max(b.maxMotorResidual, std::abs(actual.appliedTorques[wire]-expectedTorque));
                b.maxGravityResidual = std::max(b.maxGravityResidual, std::abs(gravityFD[j]-stationary.bias[vi]));
                b.maxContact = std::max(b.maxContact, std::abs(contact));
                b.maxVelocity = std::max(b.maxVelocity, std::abs(beforeV[vi]));
                b.maxAcceleration = std::max(b.maxAcceleration, std::abs(acceleration));
            }
        } else {
            if (audit) ++balanceExcluded;
            if (movingSample) ++movingExcluded;
        }
        if (t >= 0) {
            if (movingAudit && t <= rampS + .15) {
                const auto& body = world.GetBody(scene.body);
                const Vec3 commandLocalNow = Rotate(Conjugate(referenceBody.orientation), commandPoint-referenceBody.position);
                const double trackingError = Length(body.position+Rotate(body.orientation,commandLocalNow)-foot(world,scene,lifted));
                movingErrorSq += trackingError*trackingError;
                ++movingSamples;
                if (std::abs(t-rampS) < dt*.5) apexTrackingError = trackingError;
            }
            const double fy = foot(world, scene, lifted).y;
            peakLift = std::max(peakLift, fy - startY);
            contacts += d.legContactCount[lifted] > 0;
            if (d.legContactCount[lifted] == 0 && firstLoss < 0) firstLoss = t;
            minBody = std::min(minBody, world.GetBody(scene.body).position.y);
            maxBody = std::max(maxBody, world.GetBody(scene.body).position.y);
            peakW = std::max(peakW, d.maxLinkPreIntegrationAngularSpeed);
            peakTorque = std::max(peakTorque, d.peakServoTorqueUtilization);
            for (int j = 3*lifted; j < 3*lifted+3; ++j)
                maxTracking = std::max(maxTracking, std::abs(std::remainder(
                    world.GetServoJoint(ids[j]).targetAngle - world.GetServoJointAngle(ids[j]), 6.28318530718)));
        }
    }
    const auto& finalBody = world.GetBody(scene.body);
    const Vec3 finalFoot = foot(world, scene, lifted);
    const Vec3 finalLocal = Rotate(Conjugate(finalBody.orientation), finalFoot - finalBody.position);
    const double bodyDelta = finalBody.position.y - entryBody.y;
    const double rotationDelta = (Rotate(finalBody.orientation, entryLocal) - Rotate(entryOrientation, entryLocal)).y;
    const double jointDelta = Rotate(finalBody.orientation, finalLocal - entryLocal).y;
    Vec3 finalCommand = points[lifted]; finalCommand.y += .03;
    const Vec3 commandLocal = Rotate(Conjugate(referenceBody.orientation), finalCommand - referenceBody.position);
    const Vec3 finalCommandError = finalBody.position + Rotate(finalBody.orientation, commandLocal) - finalFoot;
    const double finalCommandGap = finalCommandError.y;
    std::cout << std::setprecision(10) << "{\"probe\":\"slow_liftoff\",\"pedestal\":"
              << (pedestal ? "true" : "false") << ",\"leg\":" << lifted
              << ",\"gravity_bias\":" << (gravityBias ? "true" : "false")
              << ",\"gravity_bias_start_s\":" << biasStart
              << ",\"server_stiffness_proxy\":" << (serverBias && !reportedStiffness ? "true" : "false")
              << ",\"reported_actuator_stiffness\":" << (reportedStiffness ? "true" : "false")
              << ",\"biased_contact_samples\":" << biasedContactSamples
              << ",\"target_bias_rad\":[" << targetBias[0] << ',' << targetBias[1] << ',' << targetBias[2] << ']'
              << ",\"server_stiffness_nm_per_rad\":[" << serverStiffness[0] << ',' << serverStiffness[1] << ',' << serverStiffness[2] << ']'
              << ",\"command_lift_m\":0.03,\"ramp_s\":" << rampS << ",\"peak_sphere_lift_m\":" << peakLift
              << ",\"velocity_lead\":" << (velocityLead ? "true" : "false")
              << ",\"moving_tracking_rms_m\":" << std::sqrt(movingErrorSq/std::max(1,movingSamples))
              << ",\"apex_tracking_error_m\":" << apexTrackingError
              << ",\"max_reference_rate_radps\":" << maxReferenceRate << ",\"max_target_bias_rad\":" << maxBias
              << ",\"moving_excluded_retry_samples\":" << movingExcluded
              << ",\"final_sphere_bottom_m\":" << foot(world, scene, lifted).y-radius(world, scene, lifted)
              << ",\"first_contact_loss_s\":" << firstLoss << ",\"contact_samples\":" << contacts
              << ",\"min_body_m\":" << minBody << ",\"max_body_m\":" << maxBody
              << ",\"peak_joint_error_rad\":" << maxTracking << ",\"peak_torque_utilization\":" << peakTorque
              << ",\"peak_link_w\":" << peakW << ",\"held\":" << held << ",\"recovered\":" << recovered
              << ",\"initial_command_gap_m\":" << initialCommandGap << ",\"final_command_gap_m\":" << finalCommandGap
              << ",\"final_command_error_norm_m\":" << Length(finalCommandError)
              << ",\"final_command_error_sim_world_m\":[" << finalCommandError.x << ',' << finalCommandError.y << ',' << finalCommandError.z << ']'
              << ",\"final_body_dy_m\":" << bodyDelta << ",\"final_rotation_dy_m\":" << rotationDelta
              << ",\"final_joint_dy_m\":" << jointDelta
              << ",\"final_sphere_dy_m\":" << finalFoot.y - startY
              << ",\"budget_closure_m\":" << finalFoot.y - startY - bodyDelta - rotationDelta - jointDelta
              << ",\"balance_excluded_retry_samples\":" << balanceExcluded
              << ",\"balance_contact_samples\":" << balanceContacts << ",\"balance\":[";
    bool auditOk = true;
    for (int j = 0; j < 3; ++j) {
        const auto& b = balance[j];
        if (j) std::cout << ',';
        const double n = std::max(1, b.samples);
        std::cout << "{\"wire\":" << 3*lifted+j << ",\"samples\":" << b.samples
            << ",\"error_rad\":" << b.error/n << ",\"predicted_equilibrium_error_rad\":" << b.predictedError/n
            << ",\"velocity_radps\":" << b.velocity/n << ",\"acceleration_radps2\":" << b.acceleration/n
            << ",\"kp_nm_per_rad\":" << b.kp/n << ",\"kd_nm_s_per_rad\":" << b.kd/n
            << ",\"requested_nm\":" << b.requested/n << ",\"applied_nm\":" << b.applied/n
            << ",\"gravity_nm\":" << b.gravity/n << ",\"gravity_fd_nm\":" << b.gravityFD/n
            << ",\"bias_nm\":" << b.bias/n << ",\"inertial_nm\":" << b.inertial/n
            << ",\"inferred_contact_nm\":" << b.contact/n
            << ",\"max_motor_residual_nm\":" << b.maxMotorResidual
            << ",\"max_gravity_residual_nm\":" << b.maxGravityResidual
            << ",\"max_contact_nm\":" << b.maxContact << ",\"max_velocity_radps\":" << b.maxVelocity
            << ",\"max_acceleration_radps2\":" << b.maxAcceleration << '}';
        auditOk = auditOk && b.samples > 0 && b.maxMotorResidual < 1e-10 && b.maxGravityResidual < 1e-7;
        if (pedestal) {
            const double equilibriumResidual = std::abs(b.error/n-b.predictedError/n);
            if (equilibriumResidual >= 1e-5 || b.maxContact >= 1e-8)
                std::cerr << "[liftoff-equilibrium-failure] leg=" << lifted << " joint=" << j
                          << " residual_rad=" << equilibriumResidual << " contact_nm=" << b.maxContact
                          << " velocity_radps=" << b.velocity/n << " acceleration_radps2=" << b.acceleration/n << '\n';
            auditOk = auditOk && equilibriumResidual < 1e-5 && b.maxContact < 1e-8;
        }
    }
    std::cout << "],\"moving_balance\":[";
    for (int loaded=0; loaded<2; ++loaded) for (int j=0;j<3;++j) {
        if (loaded || j) std::cout << ',';
        const auto& mb=movingBalance[loaded][j]; const double n=std::max(1,mb.samples);
        std::cout << "{\"loaded\":" << (loaded ? "true" : "false") << ",\"wire\":" << 3*lifted+j
            << ",\"samples\":" << mb.samples << ",\"mean_abs_error_rad\":" << mb.errorAbs/n
            << ",\"gravity_rad\":" << mb.gravityAbs/n << ",\"damping_rad\":" << mb.dampingAbs/n
            << ",\"inertia_rad\":" << mb.inertiaAbs/n << ",\"coriolis_rad\":" << mb.coriolisAbs/n
            << ",\"contact_rad\":" << mb.contactAbs/n << ",\"saturation_rad\":" << mb.saturationAbs/n
            << ",\"target_bias_rad\":" << mb.biasAbs/n << ",\"max_closure_rad\":" << mb.maxClosure
            << ",\"max_motor_residual_nm\":" << mb.maxMotorResidual << '}';
        auditOk = auditOk && mb.maxClosure < 1e-10 && mb.maxMotorResidual < 1e-10;
    }
    std::cout << "]}\n";
    if (pedestal && gravityBias && !serverBias) auditOk = auditOk && std::abs(finalCommandGap) < 1e-5;
    return held == 0 && balanceContacts == 0 && (biasStart == 0 || biasedContactSamples == 0)
        && std::isfinite(peakLift) && auditOk;
}
}
} // namespace minphys3d::demo
int main(int argc, char** argv) {
    bool ok = true;
    const std::string option = argc == 2 ? argv[1] : "";
    const bool movingAudit = option == "--moving-balance" || option == "--moving-gravity" || option == "--moving-lead";
    const bool velocityLead = option == "--moving-lead";
    const bool atOnset = option == "--gravity-bias-from-lift" || option == "--moving-gravity" || velocityLead;
    const bool reportedStiffness = argc == 2 && std::string(argv[1]) == "--server-actuator-stiffness";
    const bool serverBias = reportedStiffness || (argc == 2 && std::string(argv[1]) == "--server-self-weight");
    const bool gravityBias = serverBias || atOnset || (argc == 2 && std::string(argv[1]) == "--gravity-bias");
    if (argc > 1 && !gravityBias && !movingAudit) {
        std::cerr << "usage: test_pinocchio_liftoff_probe [--gravity-bias|--gravity-bias-from-lift|--server-self-weight|--server-actuator-stiffness|--moving-balance|--moving-gravity|--moving-lead]\n";
        return 2;
    }
    try {
        for (int leg : {2, 5}) for (bool pedestal : {false, true})
            ok = minphys3d::demo::run(pedestal, leg, gravityBias, atOnset ? 0.0 : 1.5, serverBias, reportedStiffness, movingAudit, velocityLead) && ok;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
    return ok ? 0 : 1;
}
