#include "minphys3d/core/world.hpp"
#include "minphys3d/core/subsystems.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace minphys3d {

namespace {

Vec3 ChooseJointReferenceVector(const Vec3& axis) {
    const Vec3 fallback = (std::abs(axis.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
    Vec3 reference = fallback - Dot(fallback, axis) * axis;
    if (!TryNormalize(reference, reference)) {
        reference = {1.0f, 0.0f, 0.0f};
    }
    return reference;
}

Vec3 ComputeLocalDirection(const Body& body, const Vec3& worldDirection) {
    return Rotate(Conjugate(Normalize(body.orientation)), worldDirection);
}

} // namespace

World::World(Vec3 gravity) : gravity_(gravity) {}

void World::SetGravity(Vec3 gravity) {
    gravity_ = gravity;
}

Vec3 World::GetGravity() const {
    return gravity_;
}

void World::SetBodyVelocityLimits(float maxLinearSpeed, float maxAngularSpeed) {
    maxBodyLinearSpeed_ = maxLinearSpeed;
    maxBodyAngularSpeed_ = maxAngularSpeed;
}

void World::SetContactSolverConfig(const ContactSolverConfig& config) {
    contactSolverConfig_ = SanitizeContactSolverConfig(config);
}

void World::SetJointSolverConfig(const JointSolverConfig& config) {
    jointSolverConfig_ = config;
}

void World::SetBroadphaseConfig(const BroadphaseConfig& config) {
    broadphaseConfig_ = config;
}

const ContactSolverConfig& World::GetContactSolverConfig() const {
    return contactSolverConfig_;
}

const JointSolverConfig& World::GetJointSolverConfig() const {
    return jointSolverConfig_;
}

const BroadphaseConfig& World::GetBroadphaseConfig() const {
    return broadphaseConfig_;
}

const BroadphaseMetrics& World::GetBroadphaseMetrics() const {
    return broadphaseMetrics_;
}

const World::PersistenceMatchDiagnostics& World::GetPersistenceMatchDiagnostics() const {
    return persistenceMatchDiagnostics_;
}

bool World::ComputeStableTangentFrame(
    const Vec3& manifoldNormal,
    const Vec3& relativeVelocity,
    Vec3& outT0,
    Vec3& outT1,
    const Vec3* preferredTangent) {
    Vec3 n{};
    if (!TryNormalize(manifoldNormal, n)) {
        return false;
    }

    auto projectToTangent = [&](const Vec3& candidate, Vec3& tangentOut) -> bool {
        const Vec3 projected = candidate - Dot(candidate, n) * n;
        return TryNormalize(projected, tangentOut);
    };

    Vec3 t0{};
    if (preferredTangent != nullptr && projectToTangent(*preferredTangent, t0)) {
        // keep previous basis when possible
    } else if (projectToTangent(relativeVelocity, t0)) {
        // align with current slip direction
    } else {
        const Vec3 fallbackAxis = (std::abs(n.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
        if (!projectToTangent(fallbackAxis, t0)) {
            return false;
        }
    }

    Vec3 t1 = Cross(n, t0);
    if (!TryNormalize(t1, t1)) {
        return false;
    }

    outT0 = t0;
    outT1 = t1;
    return true;
}

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
const World::SolverTelemetry& World::GetSolverTelemetry() const {
    return solverTelemetry_;
}

void World::SetContactPersistenceDebugLogging(bool enabled) {
    debugContactPersistence_ = enabled;
}

void World::SetBlockSolveDebugLogging(bool enabled) {
    debugBlockSolveRouting_ = enabled;
}

void World::SetDebugLogStream(std::FILE* stream) {
    debugLogStream_ = stream;
}
#endif

std::uint32_t World::CreateBody(const Body& bodyDef) {
    Body body = bodyDef;
    body.RecomputeMassProperties();
    if (body.shape == ShapeType::HalfCylinder) {
        body.position += Rotate(Normalize(body.orientation), body.centerOfMassLocal);
    }
    bodies_.push_back(body);
    shapeRevisionCounters_.push_back(0);
    shapeGeometrySignatures_.push_back(ComputeShapeGeometrySignature(body));

    BroadphaseProxy proxy;
    proxy.fatBox = ExpandAABB(body.ComputeAABB(), ComputeProxyMargin(body));
    proxy.leaf = -1;
    proxy.valid = !body.isTerrainAttachment;
    proxies_.push_back(proxy);
    if (!body.isTerrainAttachment) {
        EnsureProxyInTree(static_cast<std::uint32_t>(bodies_.size() - 1));
    }

    return static_cast<std::uint32_t>(bodies_.size() - 1);
}

void World::SetTerrainHeightfield(TerrainHeightfieldAttachment terrain) {
    if (!terrain.enabled || terrain.rows <= 1 || terrain.cols <= 1) {
        ClearTerrainHeightfield();
        return;
    }
    const std::size_t expectedCellCount = static_cast<std::size_t>(terrain.rows * terrain.cols);
    if (terrain.surfaceHeightsM.size() != expectedCellCount) {
        ClearTerrainHeightfield();
        return;
    }
    if (terrain.collisionHeightsM.empty()) {
        terrain.collisionHeightsM = terrain.surfaceHeightsM;
    }
    if (terrain.collisionHeightsM.size() != expectedCellCount) {
        ClearTerrainHeightfield();
        return;
    }
    if (!std::isfinite(terrain.cellSizeM) || terrain.cellSizeM <= 0.0f) {
        ClearTerrainHeightfield();
        return;
    }

    const bool hadAttachment = terrainAttachment_.enabled;
    const std::size_t prevCellCount = terrainAttachment_.surfaceHeightsM.size();
    std::uint64_t dirtyCells = 0;
    if (hadAttachment
        && terrainAttachment_.rows == terrain.rows
        && terrainAttachment_.cols == terrain.cols
        && std::abs(terrainAttachment_.cellSizeM - terrain.cellSizeM) <= 1.0e-6f
        && std::abs(terrainAttachment_.baseHeightM - terrain.baseHeightM) <= 1.0e-6f
        && prevCellCount == expectedCellCount) {
        const auto countDirty = [&](const std::vector<float>& previous, const std::vector<float>& current) {
            std::uint64_t count = 0;
            for (std::size_t i = 0; i < current.size(); ++i) {
                if (i >= previous.size() || std::abs(previous[i] - current[i]) > 1.0e-6f) {
                    ++count;
                }
            }
            return count;
        };
        dirtyCells = countDirty(terrainAttachment_.surfaceHeightsM, terrain.surfaceHeightsM);
        const std::vector<float>& previousCollision = terrainAttachment_.collisionHeightsM.empty()
            ? terrainAttachment_.surfaceHeightsM
            : terrainAttachment_.collisionHeightsM;
        dirtyCells = std::max(dirtyCells, countDirty(previousCollision, terrain.collisionHeightsM));
    } else {
        dirtyCells = expectedCellCount;
    }

    terrain.revision = hadAttachment ? terrainAttachment_.revision + 1u : 1u;
    terrainAttachment_ = std::move(terrain);
    terrainAttachment_.enabled = true;

    if (terrainAttachmentBodyId_ == kInvalidBodyId) {
        Body terrainBody{};
        terrainBody.shape = ShapeType::Plane;
        terrainBody.position = terrainAttachment_.centerWorld;
        terrainBody.planeNormal = terrainAttachment_.planeNormal;
        terrainBody.planeOffset = terrainAttachment_.planeHeightM;
        terrainBody.mass = 0.0f;
        terrainBody.invMass = 0.0f;
        terrainBody.restitution = 0.0f;
        terrainBody.staticFriction = 0.95f;
        terrainBody.dynamicFriction = 0.75f;
        terrainBody.isStatic = true;
        terrainBody.isSleeping = true;
        terrainBody.collisionGroup = 0x0004;
        terrainBody.collisionMask = 0;
        terrainBody.isTerrainAttachment = true;
        terrainAttachmentBodyId_ = CreateBody(terrainBody);
        bodies_[terrainAttachmentBodyId_].isTerrainAttachment = true;
        bodies_[terrainAttachmentBodyId_].collisionMask = 0;
    } else {
        Body& terrainBody = bodies_.at(terrainAttachmentBodyId_);
        terrainBody.position = terrainAttachment_.centerWorld;
        terrainBody.planeNormal = terrainAttachment_.planeNormal;
        terrainBody.planeOffset = terrainAttachment_.planeHeightM;
        terrainBody.mass = 0.0f;
        terrainBody.invMass = 0.0f;
        terrainBody.restitution = 0.0f;
        terrainBody.staticFriction = 0.95f;
        terrainBody.dynamicFriction = 0.75f;
        terrainBody.isStatic = true;
        terrainBody.isSleeping = true;
        terrainBody.collisionGroup = 0x0004;
        terrainBody.collisionMask = 0;
        terrainBody.isTerrainAttachment = true;
    }

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    if (dirtyCells > 0) {
        solverTelemetry_.terrainDirtyCellRefreshes += dirtyCells;
    }
    solverTelemetry_.terrainCacheHits += static_cast<std::uint64_t>(expectedCellCount > dirtyCells ? expectedCellCount - dirtyCells : 0u);
#else
    (void)dirtyCells;
#endif
}

void World::ClearTerrainHeightfield() {
    terrainAttachment_ = {};
}

bool World::HasTerrainHeightfield() const {
    return terrainAttachment_.enabled;
}

bool World::IsTerrainAttachmentBody(std::uint32_t bodyId) const {
    return terrainAttachmentBodyId_ != kInvalidBodyId && bodyId == terrainAttachmentBodyId_;
}

Body& World::GetBody(std::uint32_t id) {
    return bodies_.at(id);
}

const Body& World::GetBody(std::uint32_t id) const {
    return bodies_.at(id);
}

std::uint32_t World::GetBodyCount() const {
    return static_cast<std::uint32_t>(bodies_.size());
}

const ServoJoint& World::GetServoJoint(std::uint32_t id) const {
    return servoJoints_.at(id);
}

ServoJoint& World::GetServoJointMutable(std::uint32_t id) {
    return servoJoints_.at(id);
}

std::uint32_t World::GetServoJointCount() const {
    return static_cast<std::uint32_t>(servoJoints_.size());
}

float World::GetServoJointAngle(std::uint32_t id) const {
    const auto _scope = resource_profiler_.scope(
        world_resource_monitoring::toIndex(world_resource_monitoring::Section::GetServoJointAngle));
    return ComputeServoJointAngleNoProfile(servoJoints_.at(id));
}

float World::ComputeServoJointAngleNoProfile(const ServoJoint& joint) const {
    const Body& a = bodies_.at(joint.a);
    const Body& b = bodies_.at(joint.b);
    return core_internal::ComputeServoJointAngle(a, b, joint);
}

Vec3 World::ComputeLocalAnchor(std::uint32_t bodyId, const Vec3& worldAnchor) const {
    const Body& body = bodies_.at(bodyId);
    return Rotate(Conjugate(Normalize(body.orientation)), worldAnchor - body.position);
}

bool World::TryComputeLocalJointAxes(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAxis,
    Vec3& outAxisA,
    Vec3& outAxisB) const {
    Vec3 axisN{};
    if (!TryNormalize(worldAxis, axisN)) {
        outAxisA = {};
        outAxisB = {};
        return false;
    }

    outAxisA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), axisN);
    outAxisB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), axisN);
    return true;
}

std::uint32_t World::CreateDistanceJoint(std::uint32_t a, std::uint32_t b, const Vec3& worldAnchorA, const Vec3& worldAnchorB, float stiffness, float damping) {
    DistanceJoint j;
    j.a = a;
    j.b = b;
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchorA);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchorB);
    j.restLength = Length(worldAnchorB - worldAnchorA);
    j.stiffness = stiffness;
    j.damping = damping;
    joints_.push_back(j);
    return static_cast<std::uint32_t>(joints_.size() - 1);
}

std::uint32_t World::CreateHingeJoint(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAnchor,
    const Vec3& worldAxis,
    bool enableLimits,
    float lowerAngle,
    float upperAngle,
    bool enableMotor,
    float motorSpeed,
    float maxMotorTorque) {
    HingeJoint j;
    j.a = a;
    j.b = b;
    if (!TryComputeLocalJointAxes(a, b, worldAxis, j.localAxisA, j.localAxisB)) {
        return kInvalidJointId;
    }
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchor);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchor);
    Vec3 axisN{};
    (void)TryNormalize(worldAxis, axisN);
    const Vec3 worldReference = ChooseJointReferenceVector(axisN);
    j.localReferenceA = ComputeLocalDirection(bodies_.at(a), worldReference);
    j.localReferenceB = ComputeLocalDirection(bodies_.at(b), worldReference);
    j.limitsEnabled = enableLimits;
    j.lowerAngle = lowerAngle;
    j.upperAngle = upperAngle;
    j.motorEnabled = enableMotor;
    j.motorSpeed = motorSpeed;
    j.maxMotorTorque = maxMotorTorque;
    hingeJoints_.push_back(j);
    return static_cast<std::uint32_t>(hingeJoints_.size() - 1);
}

std::uint32_t World::CreateBallSocketJoint(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAnchor) {
    BallSocketJoint j;
    j.a = a;
    j.b = b;
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchor);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchor);
    ballSocketJoints_.push_back(j);
    return static_cast<std::uint32_t>(ballSocketJoints_.size() - 1);
}

std::uint32_t World::CreateFixedJoint(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAnchor) {
    FixedJoint j;
    j.a = a;
    j.b = b;
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchor);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchor);
    j.referenceRotation = Normalize(Conjugate(Normalize(bodies_.at(a).orientation)) * Normalize(bodies_.at(b).orientation));
    fixedJoints_.push_back(j);
    return static_cast<std::uint32_t>(fixedJoints_.size() - 1);
}

std::uint32_t World::CreatePrismaticJoint(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAnchor,
    const Vec3& worldAxis,
    bool enableLimits,
    float lowerTranslation,
    float upperTranslation,
    bool enableMotor,
    float motorSpeed,
    float maxMotorForce) {
    PrismaticJoint j;
    j.a = a;
    j.b = b;
    if (!TryComputeLocalJointAxes(a, b, worldAxis, j.localAxisA, j.localAxisB)) {
        return kInvalidJointId;
    }
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchor);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchor);
    j.limitsEnabled = enableLimits;
    j.lowerTranslation = lowerTranslation;
    j.upperTranslation = upperTranslation;
    j.motorEnabled = enableMotor;
    j.motorSpeed = motorSpeed;
    j.maxMotorForce = maxMotorForce;
    prismaticJoints_.push_back(j);
    return static_cast<std::uint32_t>(prismaticJoints_.size() - 1);
}

std::uint32_t World::CreateServoJoint(
    std::uint32_t a,
    std::uint32_t b,
    const Vec3& worldAnchor,
    const Vec3& worldAxis,
    float targetAngle,
    float maxServoTorque,
    float positionGain,
    float dampingGain,
    float integralGain,
    float integralClamp,
    float positionErrorSmoothing,
    float angleStabilizationScale,
    float maxServoSpeed,
    float maxCorrectionAngle) {
    ServoJoint j{};
    j.a = a;
    j.b = b;
    if (!TryComputeLocalJointAxes(a, b, worldAxis, j.localAxisA, j.localAxisB)) {
        return kInvalidJointId;
    }
    j.localAnchorA = ComputeLocalAnchor(a, worldAnchor);
    j.localAnchorB = ComputeLocalAnchor(b, worldAnchor);
    Vec3 axisN{};
    (void)TryNormalize(worldAxis, axisN);
    const Vec3 worldReference = ChooseJointReferenceVector(axisN);
    j.localReferenceA = ComputeLocalDirection(bodies_.at(a), worldReference);
    j.localReferenceB = ComputeLocalDirection(bodies_.at(b), worldReference);
    j.targetAngle = targetAngle;
    j.maxServoTorque = std::max(0.0f, maxServoTorque);
    j.positionGain = std::max(0.0f, positionGain);
    j.dampingGain = std::max(0.0f, dampingGain);
    j.integralGain = std::max(0.0f, integralGain);
    j.integralClamp = std::max(1e-5f, integralClamp);
    j.positionErrorSmoothing = std::max(0.0f, positionErrorSmoothing);
    j.maxCorrectionAngle = std::max(0.01f, maxCorrectionAngle);
    j.angleStabilizationScale = std::clamp(angleStabilizationScale, 0.0f, 1.0f);
    j.maxServoSpeed = std::max(0.0f, maxServoSpeed);
    servoJoints_.push_back(j);
    InvalidateServoAngleSampleCache();
    InvalidateServoPositionTopologyCache();
    return static_cast<std::uint32_t>(servoJoints_.size() - 1);
}

void World::InvalidateServoAngleSampleCache() {
    servoAngleSampleCacheValid_ = false;
    servoAngleSampleCache_.clear();
}

void World::InvalidateServoPositionTopologyCache() {
    servoPositionUseVelocityBiasesCacheValid_ = false;
}

void World::PrepareServoJointControlSamples() {
    if (!servoAngleSampleCacheValid_ || servoAngleSampleCache_.size() != servoJoints_.size()) {
        servoAngleSampleCache_.assign(servoJoints_.size(), 0.0f);
        for (std::size_t i = 0; i < servoJoints_.size(); ++i) {
            servoAngleSampleCache_[i] = ComputeServoJointAngleNoProfile(servoJoints_[i]);
        }
        servoAngleSampleCacheValid_ = true;
    }
    for (std::size_t i = 0; i < servoJoints_.size(); ++i) {
        ServoJoint& j = servoJoints_[i];
        const float angle = servoAngleSampleCache_[i];
        float raw = angle - j.targetAngle;
        raw = std::remainder(raw, 6.28318530717958647692f);
        if (j.positionErrorSmoothing > 0.0f) {
            const float alpha = std::min(j.positionErrorSmoothing, 1.0f);
            j.smoothedAngleError = j.smoothedAngleError + alpha * (raw - j.smoothedAngleError);
        } else {
            j.smoothedAngleError = raw;
        }
    }
}

void World::AccumulateServoAngleIntegrals(float dt) {
    // Uses the outer `World::Step` dt (not per-substep dt): integral is a slow bias into the
    // inner `SolveServoJoint` PD loop accumulated once per step.
    if (dt <= 0.0f) {
        return;
    }
    if (servoAngleSampleCache_.size() != servoJoints_.size()) {
        servoAngleSampleCache_.assign(servoJoints_.size(), 0.0f);
    }
    for (std::size_t i = 0; i < servoJoints_.size(); ++i) {
        ServoJoint& j = servoJoints_[i];
        const float angle = ComputeServoJointAngleNoProfile(j);
        servoAngleSampleCache_[i] = angle;
        if (j.integralGain <= 0.0f) {
            continue;
        }
        const float saturationRatio = std::abs(j.servoImpulseSum) / std::max(j.maxServoTorque, 1e-6f);
        if (saturationRatio > 0.95f) {
            j.integralAccum *= 0.9f;
            continue;
        }
        float err = angle - j.targetAngle;
        err = std::remainder(err, 6.28318530717958647692f);
        j.integralAccum += err * dt;
        j.integralAccum = std::clamp(j.integralAccum, -j.integralClamp, j.integralClamp);
    }
    servoAngleSampleCacheValid_ = true;
}

void World::Step(float dt, int solverIterations) {
    if (dt <= 0.0f) {
        return;
    }

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    solverTelemetry_ = {};
#endif

    const auto step_scope =
        resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::Step));
    (void)step_scope;

    AssertBodyInvariants();
    previousContacts_ = contacts_;
    previousManifolds_ = manifolds_;
    CapturePersistentPointImpulseState(previousManifolds_);

    const int substeps = ComputeSubsteps(dt);
    const float subDt = dt / static_cast<float>(substeps);

    // Servo warm-start terms are accumulated inside a single Step to help the iterative solve converge.
    // Re-using the full cached impulse state across outer Steps can inject energy into free-floating
    // articulated rigs, but hard-resetting everything every frame also makes loaded pose-hold servos
    // give up too much support torque. Keep a strongly damped fraction so each frame retains some
    // convergence history without re-applying the full stale correction.
    constexpr float kServoWarmStartDecay = 0.5f;
    for (ServoJoint& j : servoJoints_) {
        j.impulseX *= kServoWarmStartDecay;
        j.impulseY *= kServoWarmStartDecay;
        j.impulseZ *= kServoWarmStartDecay;
        j.angularImpulse1 *= kServoWarmStartDecay;
        j.angularImpulse2 *= kServoWarmStartDecay;
        j.servoImpulseSum *= kServoWarmStartDecay;
    }
    PrepareServoJointControlSamples();

    for (int stepIndex = 0; stepIndex < substeps; ++stepIndex) {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        ++debugFrameIndex_;
#endif
        currentSubstepDt_ = subDt;
        if (stepIndex == 0) {
            previousContacts_ = contacts_;
            previousManifolds_ = manifolds_;
            CapturePersistentPointImpulseState(previousManifolds_);
        }
        // Body orientation is constant from here until IntegrateOrientation() runs at the end of
        // the substep, so refresh the per-body world inverse inertia cache once and let every
        // downstream pass (IntegrateForces, warm-start, contacts, joints, PGS, relaxation) read
        // from it instead of recomputing R * I_local^-1 * R^T per call.
        RefreshBodyWorldInertias();
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::IntegrateForces));
            IntegrateForces(subDt);
            (void)scope;
        }
        BeginSplitImpulseSubstep();
        contacts_.clear();
        manifolds_.clear();
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::UpdateBroadphaseProxies));
            UpdateBroadphaseProxies();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::GenerateContacts));
            GenerateContacts();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::BuildManifolds));
            BuildManifolds();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::BuildIslands));
            BuildIslands();
            (void)scope;
        }
        PrepareIslandOrders();
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::WarmStartContacts));
            WarmStartContacts();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::WarmStartJoints));
            WarmStartJoints();
            (void)scope;
        }

        // Build per-servo-joint solver cache once per substep. All kinematic quantities
        // (anchor offsets, K^-1, axis basis, effective masses, biases) are constant across
        // PGS iterations because positions/orientations are integrated only after the loop
        // completes. SolveServoJoint then becomes a tight per-iteration impulse update.
        PrepareServoJointSolves();
        // Same idea for contacts: cache ra, rb, raCrossN, rbCrossN, normalMass once per
        // substep so the per-iteration normal-solve path (scalar / Block2 / Block4) doesn't
        // recompute the constraint Jacobian every PGS iteration.
        PrepareContactSolves();

        solverRelaxationPassActive_ = false;
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::SolveIslands));
            for (int i = 0; i < solverIterations; ++i) {
                SolveIslands();
            }
            (void)scope;
        }
        if (contactSolverConfig_.enableRelaxationPass && contactSolverConfig_.relaxationIterations > 0) {
            solverRelaxationPassActive_ = true;
            {
                const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::RelaxationPass));
                for (std::uint8_t i = 0; i < contactSolverConfig_.relaxationIterations; ++i) {
                    SolveIslands();
                }
                (void)scope;
            }
            solverRelaxationPassActive_ = false;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::ApplySplitStabilization));
            ApplySplitStabilization();
            (void)scope;
        }

        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::ResolveTOI));
            ResolveTOIPipeline(subDt);
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::IntegrateOrientation));
            IntegrateOrientation(subDt);
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::SolveJointPositions));
            const int solveStride = std::max(1, static_cast<int>(jointSolverConfig_.servoPositionSolveStride));
            const bool runPositionSolve =
                (servoPositionSolveSubstepCounter_ % static_cast<std::uint64_t>(solveStride)) == 0u;
            if (runPositionSolve) {
                SolveJointPositions();
            }
            ++servoPositionSolveSubstepCounter_;
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::PositionalCorrection));
            PositionalCorrection();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::UpdateSleeping));
            UpdateSleeping();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::ClearAccumulators));
            ClearAccumulators();
            (void)scope;
        }
        {
            const auto scope = resource_profiler_.scope(world_resource_monitoring::toIndex(world_resource_monitoring::Section::ClampBodyVelocities));
            ClampBodyVelocities();
            (void)scope;
        }
        previousContacts_ = contacts_;
        previousManifolds_ = manifolds_;
        CapturePersistentPointImpulseState(previousManifolds_);
        AssertBodyInvariants();
    }

    AccumulateServoAngleIntegrals(dt);
}

world_resource_monitoring::SectionSummary World::SnapshotResourceSections(bool reset_window) const {
    return resource_profiler_.topSections(world_resource_monitoring::kTopSectionsToReport, reset_window);
}

world_resource_monitoring::SectionSummary World::SnapshotFullResourceSections(bool reset_window) const {
    return resource_profiler_.topSections(world_resource_monitoring::kMaxSections, reset_window);
}

World::TopologySnapshot World::SnapshotTopology() const {
    TopologySnapshot snapshot{};
    snapshot.bodyCount = static_cast<std::uint32_t>(bodies_.size());
    snapshot.contactCount = static_cast<std::uint32_t>(contacts_.size());
    snapshot.manifoldCount = static_cast<std::uint32_t>(manifolds_.size());
    snapshot.islandCount = static_cast<std::uint32_t>(islands_.size());

    for (const Body& body : bodies_) {
        if (body.invMass > 0.0f) {
            ++snapshot.dynamicBodyCount;
            if (!body.isSleeping) {
                ++snapshot.awakeBodyCount;
            }
        }
    }

    for (const Island& island : islands_) {
        snapshot.maxIslandBodyCount = std::max(snapshot.maxIslandBodyCount, static_cast<std::uint32_t>(island.bodies.size()));
        snapshot.maxIslandManifoldCount = std::max(snapshot.maxIslandManifoldCount, static_cast<std::uint32_t>(island.manifolds.size()));
        const std::uint32_t jointCount = static_cast<std::uint32_t>(island.joints.size()
            + island.hinges.size()
            + island.ballSockets.size()
            + island.fixeds.size()
            + island.prismatics.size()
            + island.servos.size());
        snapshot.maxIslandJointCount = std::max(snapshot.maxIslandJointCount, jointCount);
        snapshot.maxIslandServoCount = std::max(snapshot.maxIslandServoCount, static_cast<std::uint32_t>(island.servos.size()));
    }

    return snapshot;
}

void World::AddForce(std::uint32_t id, const Vec3& force) {
    Body& b = bodies_.at(id);
    WakeBody(b);
    b.force += force;
}

void World::AddTorque(std::uint32_t id, const Vec3& torque) {
    Body& b = bodies_.at(id);
    WakeBody(b);
    b.torque += torque;
}

void World::AddForceAtPoint(std::uint32_t id, const Vec3& force, const Vec3& worldPoint) {
    Body& body = bodies_.at(id);
    WakeBody(body);
    body.force += force;
    const Vec3 r = worldPoint - body.position;
    body.torque += Cross(r, force);
}

std::size_t World::LastBroadphaseMovedProxyCount() const {
    return lastBroadphaseMovedProxyCount_;
}

std::size_t World::BroadphasePairCount() const {
    return ComputePotentialPairs().size();
}

const std::vector<Manifold>& World::DebugManifolds() const {
    return manifolds_;
}

std::size_t World::BruteForcePairCount() const {
    std::size_t count = 0;
    for (std::uint32_t i = 0; i < proxies_.size(); ++i) {
        const BroadphaseProxy& proxyA = proxies_[i];
        if (!proxyA.valid || proxyA.leaf < 0) {
            continue;
        }
        for (std::uint32_t j = i + 1; j < proxies_.size(); ++j) {
            const BroadphaseProxy& proxyB = proxies_[j];
            if (!proxyB.valid || proxyB.leaf < 0 || !Overlaps(proxyA.fatBox, proxyB.fatBox)) {
                continue;
            }
            const Body& a = bodies_[i];
            const Body& b = bodies_[j];
            if ((a.invMass == 0.0f && b.invMass == 0.0f) || (a.isSleeping && b.isSleeping)) {
                continue;
            }
            ++count;
        }
    }
    return count;
}

bool World::IsFinite(float value) {
    return std::isfinite(value);
}

bool World::IsFinite(const Vec3& v) {
    return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
}

bool World::IsFinite(const Quat& q) {
    return IsFinite(q.w) && IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z);
}

bool World::IsFinite(const Mat3& m) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (!IsFinite(m.m[row][col])) {
                return false;
            }
        }
    }
    return true;
}

bool World::IsZeroInertia(const Mat3& m) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (std::abs(m.m[row][col]) > kEpsilon) {
                return false;
            }
        }
    }
    return true;
}

bool World::TryGetPlaneNormal(const Body& plane, Vec3& outNormal) {
    return TryNormalize(plane.planeNormal, outNormal);
}

void World::AssertBodyStateFinite(const Body& body) {
    assert(IsFinite(body.position));
    assert(IsFinite(body.orientation));
    assert(IsFinite(body.velocity));
    assert(IsFinite(body.angularVelocity));
}

void World::AssertMassInertiaConsistency(const Body& body) {
    assert(IsFinite(body.mass) || body.invMass == 0.0f);
    assert(IsFinite(body.invMass));
    assert(IsFinite(body.invInertiaLocal));
    if (body.invMass == 0.0f) {
        assert(body.isStatic || !std::isfinite(body.mass));
        assert(IsZeroInertia(body.invInertiaLocal));
        return;
    }

    assert(body.mass > kEpsilon);
    assert(std::isfinite(body.mass));
    assert(std::abs(body.invMass - (1.0f / body.mass)) <= 1e-4f);
    assert(body.invInertiaLocal.m[0][0] >= -kEpsilon);
    assert(body.invInertiaLocal.m[1][1] >= -kEpsilon);
    assert(body.invInertiaLocal.m[2][2] >= -kEpsilon);
}

void World::AssertBodyInvariants() const {
    for (const Body& body : bodies_) {
        AssertBodyStateFinite(body);
        AssertMassInertiaConsistency(body);
    }
}

AABB World::ExpandAABB(const AABB& aabb, float margin) {
    AABB out = aabb;
    out.min.x -= margin;
    out.min.y -= margin;
    out.min.z -= margin;
    out.max.x += margin;
    out.max.y += margin;
    out.max.z += margin;
    return out;
}

bool World::Contains(const AABB& outer, const AABB& inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z
        && outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
}

AABB World::MergeAABB(const AABB& a, const AABB& b) {
    AABB out;
    out.min = {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)};
    out.max = {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)};
    return out;
}

float World::SurfaceArea(const AABB& aabb) {
    const Vec3 e = aabb.max - aabb.min;
    return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
}

void World::FreeNode(std::int32_t nodeId) {
    treeNodes_[nodeId].next = freeNode_;
    treeNodes_[nodeId].height = -1;
    treeNodes_[nodeId].left = -1;
    treeNodes_[nodeId].right = -1;
    treeNodes_[nodeId].parent = -1;
    treeNodes_[nodeId].bodyId = -1;
    freeNode_ = nodeId;
}

int World::ComputeSubsteps(float dt) const {
    float maxRatio = 0.0f;
    for (const Body& body : bodies_) {
        if (body.invMass == 0.0f || body.isSleeping) {
            continue;
        }
        float characteristic = 1.0f;
        if (body.shape == ShapeType::Sphere) {
            characteristic = std::max(body.radius, 0.05f);
        } else if (body.shape == ShapeType::Box) {
            characteristic = std::max({body.halfExtents.x, body.halfExtents.y, body.halfExtents.z, 0.05f});
        } else if (body.shape == ShapeType::Capsule) {
            characteristic = std::max(body.radius, 0.05f);
        } else if (body.shape == ShapeType::Cylinder || body.shape == ShapeType::HalfCylinder) {
            characteristic = std::max({body.radius, body.halfHeight, 0.05f});
        } else if (body.shape == ShapeType::Compound) {
            const AABB bounds = body.ComputeAABB();
            const Vec3 halfExtents = 0.5f * (bounds.max - bounds.min);
            characteristic = std::max({halfExtents.x, halfExtents.y, halfExtents.z, 0.05f});
        }
        const float travel = Length(body.velocity) * dt;
        maxRatio = std::max(maxRatio, travel / (characteristic * kMaxSubstepDistanceFactor));
    }
    return std::clamp(static_cast<int>(std::ceil(std::max(1.0f, maxRatio))), 1, 8);
}

} // namespace minphys3d
