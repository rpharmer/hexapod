#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifndef MINPHYS3D_SOLVER_TELEMETRY_ENABLED
#if !defined(NDEBUG) || defined(MINPHYS3D_ENABLE_SOLVER_TELEMETRY)
#define MINPHYS3D_SOLVER_TELEMETRY_ENABLED 1
#else
#define MINPHYS3D_SOLVER_TELEMETRY_ENABLED 0
#endif
#endif

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
#include <cstdio>
#endif

#include "minphys3d/broadphase/types.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world_resource_monitoring.hpp"
#include "minphys3d/core/world_types.hpp"
#include "minphys3d/joints/types.hpp"
#include "minphys3d/narrowphase/epa.hpp"
#include "minphys3d/narrowphase/gjk.hpp"
#include "minphys3d/narrowphase/dispatch.hpp"
#include "minphys3d/solver/types.hpp"

namespace minphys3d {

class World {
public:
    static constexpr std::uint32_t kInvalidJointId = std::numeric_limits<std::uint32_t>::max();
    static constexpr std::uint32_t kInvalidBodyId = std::numeric_limits<std::uint32_t>::max();

    struct TerrainHeightfieldAttachment {
        bool enabled = false;
        std::uint64_t revision = 0;
        int rows = 0;
        int cols = 0;
        float cellSizeM = 0.0f;
        Vec3 gridOriginWorld{};
        Vec3 centerWorld{};
        Vec3 planeNormal{0.0f, 1.0f, 0.0f};
        float planeHeightM = 0.0f;
        float baseHeightM = 0.0f;
        bool useConservativeCollision = false;
        std::vector<float> surfaceHeightsM{};
        std::vector<float> collisionHeightsM{};
    };

    explicit World(Vec3 gravity = {0.0f, -9.81f, 0.0f});

    void SetGravity(Vec3 gravity);
    Vec3 GetGravity() const;

    /// Caps dynamic body linear / angular speed magnitudes at the end of each physics substep (after
    /// contacts, joints, TOI, and orientation integration). Non-positive disables that component.
    void SetBodyVelocityLimits(float maxLinearSpeed, float maxAngularSpeed);
    float GetMaxBodyLinearSpeed() const { return maxBodyLinearSpeed_; }
    float GetMaxBodyAngularSpeed() const { return maxBodyAngularSpeed_; }

    void SetContactSolverConfig(const ContactSolverConfig& config);
    void SetJointSolverConfig(const JointSolverConfig& config);
    void SetBroadphaseConfig(const BroadphaseConfig& config);

    const ContactSolverConfig& GetContactSolverConfig() const;
    const JointSolverConfig& GetJointSolverConfig() const;
    const BroadphaseConfig& GetBroadphaseConfig() const;
    const BroadphaseMetrics& GetBroadphaseMetrics() const;
    void SetTerrainHeightfield(TerrainHeightfieldAttachment terrain);
    void ClearTerrainHeightfield();
    bool HasTerrainHeightfield() const;
    bool IsTerrainAttachmentBody(std::uint32_t bodyId) const;
    struct PersistenceMatchDiagnostics {
        std::uint64_t matchedPoints = 0;
        std::uint64_t droppedPoints = 0;
        std::uint64_t newPoints = 0;
        float churnRatio = 0.0f;
    };
    const PersistenceMatchDiagnostics& GetPersistenceMatchDiagnostics() const;

    struct TopologySnapshot {
        std::uint32_t bodyCount = 0;
        std::uint32_t dynamicBodyCount = 0;
        std::uint32_t awakeBodyCount = 0;
        std::uint32_t contactCount = 0;
        std::uint32_t manifoldCount = 0;
        std::uint32_t islandCount = 0;
        std::uint32_t maxIslandBodyCount = 0;
        std::uint32_t maxIslandManifoldCount = 0;
        std::uint32_t maxIslandJointCount = 0;
        std::uint32_t maxIslandServoCount = 0;
    };
    [[nodiscard]] TopologySnapshot SnapshotTopology() const;
    void SetNarrowphaseDispatchPolicy(NarrowphaseDispatchPolicy policy);
    NarrowphaseDispatchPolicy GetNarrowphaseDispatchPolicy() const;
    static bool ComputeStableTangentFrame(
        const Vec3& manifoldNormal,
        const Vec3& relativeVelocity,
        Vec3& outT0,
        Vec3& outT1,
        const Vec3* preferredTangent = nullptr);

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    struct SolverTelemetry {
        struct FallbackReasonCounters {
            std::uint64_t none = 0;
            std::uint64_t ineligible = 0;
            std::uint64_t persistenceGate = 0;
            std::uint64_t invalidManifoldNormal = 0;
            std::uint64_t contactNormalMismatch = 0;
            std::uint64_t missingBlockSlots = 0;
            std::uint64_t degenerateMassMatrix = 0;
            std::uint64_t conditionEstimateExceeded = 0;
            std::uint64_t lcpFailure = 0;
            std::uint64_t nonFiniteResult = 0;
        };

        struct ManifoldSolveBucket {
            std::uint64_t solveCount = 0;
            std::uint64_t manifoldContactCount = 0;
            std::uint64_t selectedBlockSize = 0;
            std::uint64_t blockUsed = 0;
            std::uint64_t fallbackUsed = 0;
            FallbackReasonCounters fallbackReason{};
            double determinantOrConditionEstimate = 0.0;
            std::uint64_t determinantOrConditionEstimateSamples = 0;
            double impulseContinuityMetric = 0.0;
            std::uint64_t impulseContinuityMetricSamples = 0;
        };

        std::uint64_t blockSolveEligible = 0;
        std::uint64_t blockSolveUsed = 0;
        std::uint64_t scalarPathIneligible = 0;
        std::uint64_t scalarFallbackPersistenceGate = 0;
        std::uint64_t scalarFallbackInvalidNormal = 0;
        std::uint64_t scalarFallbackNormalMismatch = 0;
        std::uint64_t scalarFallbackMissingSlots = 0;
        std::uint64_t scalarFallbackDegenerateSystem = 0;
        std::uint64_t scalarFallbackConditionEstimate = 0;
        std::uint64_t scalarFallbackLcpFailure = 0;
        std::uint64_t scalarFallbackNonFinite = 0;
        std::uint64_t reorderDetected = 0;
        std::uint64_t featureIdChurnEvents = 0;
        std::uint64_t topologyChangeEvents = 0;
        std::uint64_t impulseResetPoints = 0;
        std::uint64_t selectedPairOscillationEvents = 0;
        std::uint64_t manifoldPointAdds = 0;
        std::uint64_t manifoldPointRemoves = 0;
        std::uint64_t manifoldPointReorders = 0;
        std::uint64_t manifoldQualityLow = 0;
        std::uint64_t manifoldQualityMedium = 0;
        std::uint64_t manifoldQualityHigh = 0;
        std::uint64_t tangentBasisResets = 0;
        std::uint64_t tangentBasisReused = 0;
        std::uint64_t tangentImpulseReprojected = 0;
        std::uint64_t tangentImpulseReset = 0;
        std::uint64_t manifoldFrictionBudgetSaturated = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedSelectedPair = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedAllContacts = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedBlended = 0;
        std::uint64_t blockRejectedByTypePolicy = 0;
        std::uint64_t blockRejectedByQualityOrPersistence = 0;
        std::uint64_t face4Attempted = 0;
        std::uint64_t face4Used = 0;
        std::uint64_t face4FallbackToBlock2 = 0;
        std::uint64_t face4FallbackToScalar = 0;
        std::uint64_t face4BlockedByFrictionCoherenceGate = 0;
        std::uint64_t anchorReuseHitCount = 0;
        std::uint64_t anchorReuseFallbackCount = 0;
        std::uint64_t supportDepthOrderApplied = 0;
        std::uint64_t supportDepthOrderBypassed = 0;
        std::uint64_t jointBlockSolveUsed = 0;
        std::uint64_t jointBlockFallbackDegenerate = 0;
        std::uint64_t jointBlockFallbackConditionEstimate = 0;
        std::uint64_t jointBlockFallbackNonFinite = 0;
        std::uint64_t epaFallbackUsed = 0;
        std::uint64_t epaIterationBailout = 0;
        std::uint64_t epaDegenerateFaces = 0;
        std::uint64_t epaDuplicateSupports = 0;
        std::uint64_t terrainContactAdds = 0;
        std::uint64_t terrainCellsTested = 0;
        std::uint64_t terrainContactsEmitted = 0;
        std::uint64_t terrainManifoldMerges = 0;
        std::uint64_t terrainCacheHits = 0;
        std::uint64_t terrainDirtyCellRefreshes = 0;

        ManifoldSolveBucket manifoldSolveScope{};
        std::unordered_map<std::uint8_t, ManifoldSolveBucket> manifoldTypeBuckets{};
    };

    const SolverTelemetry& GetSolverTelemetry() const;

    void SetContactPersistenceDebugLogging(bool enabled);

    void SetBlockSolveDebugLogging(bool enabled);

    void SetDebugLogStream(std::FILE* stream);
#endif

    std::uint32_t CreateBody(const Body& bodyDef);

    Body& GetBody(std::uint32_t id);

    const Body& GetBody(std::uint32_t id) const;

    std::uint32_t GetBodyCount() const;

    const ServoJoint& GetServoJoint(std::uint32_t id) const;
    ServoJoint& GetServoJointMutable(std::uint32_t id);

    std::uint32_t GetServoJointCount() const;

    float GetServoJointAngle(std::uint32_t id) const;

    std::uint32_t CreateDistanceJoint(std::uint32_t a, std::uint32_t b, const Vec3& worldAnchorA, const Vec3& worldAnchorB, float stiffness = 1.0f, float damping = 0.1f);

    std::uint32_t CreateHingeJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor,
        const Vec3& worldAxis = {0.0f, 1.0f, 0.0f},
        bool enableLimits = false,
        float lowerAngle = 0.0f,
        float upperAngle = 0.0f,
        bool enableMotor = false,
        float motorSpeed = 0.0f,
        float maxMotorTorque = 0.0f);
    std::uint32_t CreateBallSocketJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor);
    std::uint32_t CreateFixedJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor);
    std::uint32_t CreatePrismaticJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor,
        const Vec3& worldAxis = {1.0f, 0.0f, 0.0f},
        bool enableLimits = false,
        float lowerTranslation = 0.0f,
        float upperTranslation = 0.0f,
        bool enableMotor = false,
        float motorSpeed = 0.0f,
        float maxMotorForce = 0.0f);
    std::uint32_t CreateServoJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor,
        const Vec3& worldAxis = {0.0f, 1.0f, 0.0f},
        float targetAngle = 0.0f,
        float maxServoTorque = 1.0f,
        float positionGain = 40.0f,
        float dampingGain = 1.0f,
        float integralGain = 0.0f,
        float integralClamp = 0.5f,
        float positionErrorSmoothing = 0.0f,
        float angleStabilizationScale = 1.0f,
        float maxServoSpeed = 0.0f,
        float maxCorrectionAngle = 0.5f);

    void Step(float dt, int solverIterations = 8);
    [[nodiscard]] world_resource_monitoring::SectionSummary SnapshotResourceSections(bool reset_window = true) const;

    void AddForce(std::uint32_t id, const Vec3& force);

    void AddTorque(std::uint32_t id, const Vec3& torque);

    void AddForceAtPoint(std::uint32_t id, const Vec3& force, const Vec3& worldPoint);

    std::size_t LastBroadphaseMovedProxyCount() const;

    std::size_t BroadphasePairCount() const;


    const std::vector<Manifold>& DebugManifolds() const;

    std::size_t BruteForcePairCount() const;

private:
    static constexpr float kQuaternionNormalizationTolerance = 1e-3f;
    static constexpr std::size_t kMaxContactsPerManifold = 4;
    static constexpr float kWakeContactRelativeSpeedThreshold = 0.35f;
    static constexpr float kWakeContactPenetrationThreshold = 0.02f;
    static constexpr float kWakeJointRelativeSpeedThreshold = 0.15f;
    static constexpr float kPersistenceLocalAnchorDriftThreshold = 0.06f;
    static constexpr float kPersistenceWorldAnchorDriftThreshold = 0.08f;

    static bool IsFinite(float value);
    static bool IsFinite(const Vec3& v);
    static bool IsFinite(const Quat& q);
    static bool IsFinite(const Mat3& m);
    Vec3 ComputeLocalAnchor(std::uint32_t bodyId, const Vec3& worldAnchor) const;
    bool TryComputeLocalJointAxes(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAxis,
        Vec3& outAxisA,
        Vec3& outAxisB) const;
    static std::uint64_t ComputeShapeGeometrySignature(const Body& body);
    void RefreshShapeRevisionCounters();
    bool ConvexOverlapWithCache(std::uint32_t a, std::uint32_t b);
    void EmitConvexManifoldSeeds();
    static bool IsZeroInertia(const Mat3& m);
    static bool TryGetPlaneNormal(const Body& plane, Vec3& outNormal);
    static void AssertBodyStateFinite(const Body& body);
    static void AssertMassInertiaConsistency(const Body& body);
    void AssertBodyInvariants() const;

    static void AssertQuaternionNormalized(const Quat& q);

    static AABB ExpandAABB(const AABB& aabb, float margin);
    static bool Contains(const AABB& outer, const AABB& inner);
    static AABB MergeAABB(const AABB& a, const AABB& b);
    static float SurfaceArea(const AABB& aabb);
    void UpdateBroadphaseProxies();
    float ComputeProxyMargin(const Body& body) const;
    void ReinsertProxy(std::uint32_t bodyId);
    void PartialRebuildBroadphase();
    void FullRebuildBroadphase();
    void UpdateBroadphaseQualityMetrics();
    void MaybeTriggerBroadphaseRebuild();
    void EnsureProxyInTree(std::uint32_t bodyId);
    bool IsPairEligible(std::uint32_t a, std::uint32_t b) const;
    std::uint64_t MakePairKey(std::uint32_t a, std::uint32_t b) const;

    std::int32_t AllocateNode();

    void FreeNode(std::int32_t nodeId);

    std::int32_t InsertLeaf(std::uint32_t bodyId, const AABB& fatBox);

    void RemoveLeaf(std::int32_t leaf);

    std::int32_t Balance(std::int32_t iA);

    int ComputeSubsteps(float dt) const;

    struct TOIEvent {
        bool hit = false;
        float toi = 0.0f;
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        Vec3 normal{0.0f, 1.0f, 0.0f};
        Vec3 point{0.0f, 0.0f, 0.0f};
    };

    static float ClosestPointParameter(const Vec3& a, const Vec3& b, const Vec3& p);

    static float SmoothStep01(float t);

    float EffectiveRestitutionCutoffSpeed() const;

    float ComputeRestitution(float speedIntoContact, float restitutionA, float restitutionB) const;

    float ComputeHighMassRatioBoost(const Body& a, const Body& b) const;

    void AdvanceDynamicBodies(float dt);

    void ResolveTOIImpact(const TOIEvent& hit);

    TOIEvent SweepSpherePlane(std::uint32_t sphereId, std::uint32_t planeId, float maxDt) const;

    TOIEvent SweepSphereBox(std::uint32_t sphereId, std::uint32_t boxId, float maxDt) const;

    TOIEvent SweepSphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId, float maxDt) const;

    TOIEvent FindEarliestTOI(float maxDt) const;

    void ResolveTOIPipeline(float dt);

    static std::pair<std::uint32_t, std::uint32_t> CanonicalBodyOrder(std::uint32_t a, std::uint32_t b);

    static std::uint64_t CanonicalFeaturePairId(
        std::uint32_t a,
        std::uint32_t b,
        std::uint32_t featureOnA,
        std::uint32_t featureOnB,
        std::uint16_t detail = 0u);

    static ContactKey MakeContactKey(
        std::uint32_t a,
        std::uint32_t b,
        std::uint8_t manifoldType,
        std::uint64_t canonicalFeatureId);

    static std::uint64_t ContactKeyStableValue(const ContactKey& key);

    static ManifoldKey MakeManifoldId(std::uint32_t a, std::uint32_t b, std::uint8_t manifoldType);

    static PersistentPointKey MakePersistentPointKey(const ManifoldKey& manifoldId, std::uint64_t canonicalFeatureId, std::uint8_t ordinal);

    static void WakeBody(Body& body);

    void WakeConnectedBodies(std::uint32_t start);

    static float ProjectBoxOntoAxis(const Body& box, const Vec3& axis);

    static Vec3 ClosestPointOnBox(const Body& box, const Vec3& worldPoint);

    void IntegrateForces(float dt);

    void IntegrateVelocities(float dt);

    void IntegrateOrientation(float dt);

    void AddContact(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& normal,
        const Vec3& point,
        float penetration,
        std::uint8_t manifoldType = 0,
        std::uint64_t canonicalFeatureId = 0);

    static int FindBlockSlot(const Manifold& manifold, std::uint64_t contactKey);

    static std::array<float, 3>* FindPerContactImpulseCache(Manifold& manifold, std::uint64_t contactKey);

    static const std::array<float, 3>* FindPerContactImpulseCache(const Manifold& manifold, std::uint64_t contactKey);

    static std::array<float, 3>& EnsurePerContactImpulseCache(Manifold& manifold, std::uint64_t contactKey);

    static int FindFirstFreeBlockSlot(const std::array<bool, 2>& slotOccupied);

    static int EnsureBlockSlotForContact(
        Manifold& manifold,
        const Contact& contact,
        std::array<bool, 2>& slotOccupied);

    static void RefreshManifoldBlockCache(Manifold& manifold);

    static std::uint64_t StableContactFallbackKey(const Contact& contact);

    static bool ContactComesBefore(const Contact& lhs, const Contact& rhs);

    static void SortManifoldContacts(std::vector<Contact>& contacts);

    struct ManifoldQualityScore {
        float penetration = 0.0f;
        float spreadArea = 0.0f;
        float normalCoherence = 0.0f;
        float total = 0.0f;
    };

    static ManifoldQualityScore ComputeManifoldQualityScore(const Manifold& manifold);

    static void ReduceManifoldToMaxPoints(Manifold& manifold, std::size_t maxPoints = 4);

    class ManifoldManager {
    public:
        explicit ManifoldManager(World& world)
            : world_(world) {}

        void Process(Manifold& manifold, const Manifold* previous) const;

    private:
        World& world_;
    };

    void ManageManifoldContacts(Manifold& manifold, const Manifold* previous);

    void BuildManifolds();

    struct PersistentPointImpulseState {
        float normalImpulseSum = 0.0f;
        float tangentImpulseSum0 = 0.0f;
        float tangentImpulseSum1 = 0.0f;
        std::uint16_t persistenceAge = 0;
        bool anchorsValid = false;
        Vec3 localAnchorA{0.0f, 0.0f, 0.0f};
        Vec3 localAnchorB{0.0f, 0.0f, 0.0f};
        Vec3 worldPoint{0.0f, 0.0f, 0.0f};
    };

    struct PersistentPointMatchCandidate {
        PersistentPointKey key{};
        PersistentPointImpulseState state{};
        float localAnchorDriftSq = 0.0f;
        float worldAnchorDriftSq = 0.0f;
    };

    static bool PersistentPointCandidateLess(
        const PersistentPointMatchCandidate& lhs,
        const PersistentPointMatchCandidate& rhs);

    static bool TryMatchPersistentPoint(
        const std::unordered_map<PersistentPointKey, PersistentPointImpulseState, PersistentPointKeyHash>& previousState,
        const ManifoldKey& manifoldId,
        const Contact& contact,
        const std::unordered_set<PersistentPointKey, PersistentPointKeyHash>& usedKeys,
        PersistentPointMatchCandidate& outCandidate);

    void CapturePersistentPointImpulseState(const std::vector<Manifold>& manifolds);

    void BuildIslands();

    std::vector<Pair> ComputePotentialPairs() const;

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    std::vector<Pair> ComputePotentialPairsBruteForce() const;

    static std::unordered_set<std::uint64_t> PairSetFromPairs(const std::vector<Pair>& pairs);
#endif

    void GenerateContacts();
    void GenerateTerrainContacts();
    static bool IsConvexShape(ShapeType shape);
    ConvexDispatchRoute SelectConvexDispatchRoute(ShapeType a, ShapeType b) const;

    static Vec3 ClampPointToExtents(const Vec3& p, const Vec3& extents);

    static Vec3 StableDirection(const Vec3& primary, const std::array<Vec3, 4>& fallbacks);

    static std::pair<float, float> ClosestSegmentParameters(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2);

    struct SegmentBoxClosest {
        float t = 0.0f;
        Vec3 segmentPoint{};
        Vec3 boxPoint{};
        float distSq = 0.0f;
    };

    static SegmentBoxClosest ClosestSegmentPointToBox(const Vec3& segA, const Vec3& segB, const Vec3& extents);

    void SphereSphere(std::uint32_t ia, std::uint32_t ib);

    void ConvexPlane(std::uint32_t convexBodyId, std::uint32_t planeId);

    void ConvexConvexEPA(std::uint32_t bodyAId, std::uint32_t bodyBId);

    void SphereCylinder(std::uint32_t sphereId, std::uint32_t cylId);

    void CylinderBox(std::uint32_t cylId, std::uint32_t boxId);

    void CylinderCylinder(std::uint32_t aId, std::uint32_t bId);

    void CapsuleCylinder(std::uint32_t capsuleId, std::uint32_t cylId);

    void SphereHalfCylinder(std::uint32_t sphereId, std::uint32_t halfCylinderId);

    void HalfCylinderBox(std::uint32_t halfCylinderId, std::uint32_t boxId);

    void HalfCylinderCylinder(std::uint32_t halfCylinderId, std::uint32_t cylinderId);

    void HalfCylinderHalfCylinder(std::uint32_t aId, std::uint32_t bId);

    void CapsuleHalfCylinder(std::uint32_t capsuleId, std::uint32_t halfCylinderId);

    void SpherePlane(std::uint32_t sphereId, std::uint32_t planeId);

    void CapsulePlane(std::uint32_t capsuleId, std::uint32_t planeId);

    void SphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId);

    void CapsuleCapsule(std::uint32_t aId, std::uint32_t bId);

    void CapsuleBox(std::uint32_t capsuleId, std::uint32_t boxId);

    void BoxPlane(std::uint32_t boxId, std::uint32_t planeId);

    void SphereBox(std::uint32_t sphereId, std::uint32_t boxId);

    void BoxBox(std::uint32_t aId, std::uint32_t bId);

    void WarmStartContacts();

    void WarmStartJoints();

    void SolveNormalScalar(Contact& c);

    enum class BlockSolveFallbackReason {
        None,
        Ineligible,
        TypePolicy,
        QualityGate,
        PersistenceGate,
        InvalidManifoldNormal,
        ContactNormalMismatch,
        MissingBlockSlots,
        DegenerateMassMatrix,
        ConditionEstimateExceeded,
        LcpFailure,
        NonFiniteResult,
    };

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    static void ResetBlockSolveDebugStep(Manifold& manifold);

    static void IncrementBlockSolveFallbackCounter(Manifold& manifold, BlockSolveFallbackReason reason);
#endif
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    void IncrementFallbackReasonTelemetry(SolverTelemetry::FallbackReasonCounters& counters, BlockSolveFallbackReason reason);

    void RecordManifoldSolveTelemetry(const Manifold& manifold,
                                      BlockSolveFallbackReason fallbackReason,
                                      float determinantOrConditionEstimate,
                                      float impulseContinuityMetric);
#endif

    static bool IsValidBlockContactPoint(const Contact& contact);

    static bool ContactPairStableOrder(const Contact& lhs, const Contact& rhs);

    static std::array<std::uint64_t, 2> SortedContactKeyPair(std::uint64_t k0, std::uint64_t k1);

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    struct SelectionHistory {
        std::array<std::uint64_t, 2> lastPair{0u, 0u};
        std::array<std::uint64_t, 2> previousPair{0u, 0u};
        bool hasLast = false;
        bool hasPrevious = false;
    };

    void RecordSelectedPairHistory(const Manifold& manifold);

    void DebugLogManifoldContactKeys(const Manifold& manifold) const;

    void DebugLogContactTransitions(const Manifold& previous, const Manifold& current) const;
#endif

    void SelectBlockSolvePair(Manifold& manifold) const;

    bool IsBlockSolveEligible(const Manifold& manifold, BlockSolveFallbackReason* outIneligibleReason = nullptr) const;

    bool EnsureStableTwoPointOrder(Manifold& manifold);

    bool IsFace4PointBlockEligible(const Manifold& manifold, BlockSolveFallbackReason* outReason = nullptr);

    bool SolveNormalBlock2(Manifold& manifold, BlockSolveFallbackReason& fallbackReason, float& determinantOrConditionEstimate);
    bool SolveNormalProjected4(Manifold& manifold, BlockSolveFallbackReason& fallbackReason, float& conditionEstimate);

    void SolveManifoldNormalImpulses(Manifold& manifold);

    void SolveContactsInManifold(Manifold& manifold);

    void SolveDistanceJoint(DistanceJoint& j);
    enum class JointBlockFallbackReason {
        None,
        DegenerateMassMatrix,
        ConditionEstimateExceeded,
        NonFiniteResult,
    };
    bool SolveHingeAnchorBlock3x3(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& ra,
        const Vec3& rb,
        const Vec3& error,
        const Vec3& relVel,
        Vec3& outLambda,
        JointBlockFallbackReason& outReason) const;

    void SolveBallSocketJoint(BallSocketJoint& j);
    void SolveHingeJoint(HingeJoint& j);
    void SolveFixedJoint(FixedJoint& j);
    void SolvePrismaticJoint(PrismaticJoint& j);
    void SolveServoJoint(ServoJoint& j);

    void PrepareIslandOrders();

    void SolveIslands();

    void SolveJointPositions();
    void BeginSplitImpulseSubstep();
    void ApplySplitStabilization();
    void AccumulateSplitImpulseCorrection(std::uint32_t bodyId, const Vec3& linearDelta, const Vec3& angularDelta);
    bool TryComputeAnchorSeparation(const Contact& c, float& outPenetration) const;

    void ApplyImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& ra,
        const Vec3& rb,
        const Vec3& impulse);

    void ApplyAngularImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& angularImpulse);

    void PositionalCorrection();

    void UpdateSleeping();

    void ClearAccumulators();
    void ClampBodyVelocities();

private:
    void PrepareServoJointControlSamples();
    void AccumulateServoAngleIntegrals(float dt);

    struct NarrowphaseCacheKey {
        std::uint32_t loBody = 0;
        std::uint32_t hiBody = 0;
        std::uint32_t loRevision = 0;
        std::uint32_t hiRevision = 0;

        bool operator==(const NarrowphaseCacheKey& other) const {
            return loBody == other.loBody
                && hiBody == other.hiBody
                && loRevision == other.loRevision
                && hiRevision == other.hiRevision;
        }
    };

    struct NarrowphaseCacheKeyHash {
        std::size_t operator()(const NarrowphaseCacheKey& key) const noexcept {
            std::size_t seed = static_cast<std::size_t>(key.loBody);
            seed ^= static_cast<std::size_t>(key.hiBody) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            seed ^= static_cast<std::size_t>(key.loRevision) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            seed ^= static_cast<std::size_t>(key.hiRevision) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            return seed;
        }
    };

    struct ConvexSeedKey {
        std::uint32_t loBody = 0;
        std::uint32_t hiBody = 0;

        bool operator==(const ConvexSeedKey& other) const {
            return loBody == other.loBody && hiBody == other.hiBody;
        }
    };

    struct ConvexSeedKeyHash {
        std::size_t operator()(const ConvexSeedKey& key) const noexcept {
            std::size_t seed = static_cast<std::size_t>(key.loBody);
            seed ^= static_cast<std::size_t>(key.hiBody) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            return seed;
        }
    };

    Vec3 gravity_{};
    float maxBodyLinearSpeed_ = 120.0f;
    float maxBodyAngularSpeed_ = 180.0f;
    ContactSolverConfig contactSolverConfig_{};
    JointSolverConfig jointSolverConfig_{};
    BroadphaseConfig broadphaseConfig_{};
    mutable BroadphaseMetrics broadphaseMetrics_{};
    std::uint64_t broadphaseStepCounter_ = 0;
    std::uint64_t lastBroadphaseRebuildStep_ = 0;
    NarrowphaseDispatchPolicy narrowphaseDispatchPolicy_ = NarrowphaseDispatchPolicy::PreferSpecializedFastPaths;
    float currentSubstepDt_ = 1.0f / 60.0f;
    bool solverRelaxationPassActive_ = false;
    std::vector<Body> bodies_;
    std::vector<std::uint32_t> shapeRevisionCounters_;
    std::vector<std::uint64_t> shapeGeometrySignatures_;
    std::vector<Vec3> splitLinearPositionDelta_;
    std::vector<Vec3> splitAngularPositionDelta_;
    std::vector<BroadphaseProxy> proxies_;
    std::vector<TreeNode> treeNodes_;
    std::int32_t rootNode_ = -1;
    std::int32_t freeNode_ = -1;
    std::size_t lastBroadphaseMovedProxyCount_ = 0;
    std::vector<std::uint32_t> movedProxyIds_;
    mutable std::vector<Pair> cachedPotentialPairs_{};
    mutable std::unordered_set<std::uint64_t> cachedPotentialPairKeySet_{};
    mutable std::vector<std::uint8_t> previousBodyActiveState_{};
    std::vector<Contact> contacts_;
    std::vector<Contact> previousContacts_;
    std::vector<Manifold> manifolds_;
    std::vector<Manifold> previousManifolds_;
    world_resource_monitoring::Profiler resource_profiler_{world_resource_monitoring::kSectionLabels};
    std::vector<Island> islands_;
    std::vector<IslandOrderResult> islandOrders_;
    std::vector<DistanceJoint> joints_;
    std::vector<HingeJoint> hingeJoints_;
    std::vector<BallSocketJoint> ballSocketJoints_;
    std::vector<FixedJoint> fixedJoints_;
    std::vector<PrismaticJoint> prismaticJoints_;
    std::vector<ServoJoint> servoJoints_;
    std::unordered_map<PersistentPointKey, PersistentPointImpulseState, PersistentPointKeyHash> persistentPointImpulses_;
    PersistenceMatchDiagnostics persistenceMatchDiagnostics_{};
    std::unordered_map<NarrowphaseCacheKey, NarrowphaseCache, NarrowphaseCacheKeyHash> narrowphaseCache_;
    std::unordered_map<ConvexSeedKey, EpaPenetrationResult, ConvexSeedKeyHash> convexManifoldSeeds_;
    TerrainHeightfieldAttachment terrainAttachment_{};
    std::uint32_t terrainAttachmentBodyId_ = kInvalidBodyId;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    std::FILE* DebugLogStream() const {
        return debugLogStream_ != nullptr ? debugLogStream_ : stderr;
    }

    SolverTelemetry solverTelemetry_{};
    bool debugContactPersistence_ = false;
    bool debugBlockSolveRouting_ = false;
    std::FILE* debugLogStream_ = stderr;
    std::uint64_t debugFrameIndex_ = 0;
    std::unordered_map<ManifoldKey, SelectionHistory, ManifoldKeyHash> selectedPairHistory_;
#endif
};


} // namespace minphys3d
