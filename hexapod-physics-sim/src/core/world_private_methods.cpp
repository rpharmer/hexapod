#include "minphys3d/core/world.hpp"

namespace minphys3d {

void World::AssertQuaternionNormalized(const Quat& q) {

        const float lenSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        assert(IsFinite(lenSq));
        assert(std::abs(lenSq - 1.0f) <= kQuaternionNormalizationTolerance);
    }

float World::ClosestPointParameter(const Vec3& a, const Vec3& b, const Vec3& p) {

        const Vec3 ab = b - a;
        const float denom = Dot(ab, ab);
        if (denom <= kEpsilon) {
            return 0.0f;
        }
        return std::clamp(Dot(p - a, ab) / denom, 0.0f, 1.0f);
    }

float World::SmoothStep01(float t) {

        const float x = std::clamp(t, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }

float World::EffectiveRestitutionCutoffSpeed() const {

        return std::max({
            0.0f,
            contactSolverConfig_.bounceVelocityThreshold,
            contactSolverConfig_.restitutionSuppressionSpeed,
            contactSolverConfig_.restitutionVelocityCutoff,
        });
    }

float World::ComputeRestitution(float speedIntoContact, float restitutionA, float restitutionB) const {

        if (speedIntoContact <= 0.0f || speedIntoContact < EffectiveRestitutionCutoffSpeed()) {
            return 0.0f;
        }
        return std::clamp(std::min(restitutionA, restitutionB), 0.0f, 1.0f);
    }

float World::ComputeHighMassRatioBoost(const Body& a, const Body& b) const {

        if (a.invMass <= 0.0f || b.invMass <= 0.0f) {
            return 1.0f;
        }
        const float massA = 1.0f / a.invMass;
        const float massB = 1.0f / b.invMass;
        const float massRatio = std::max(massA, massB) / std::max(std::min(massA, massB), kEpsilon);
        if (massRatio <= contactSolverConfig_.highMassRatioThreshold) {
            return 1.0f;
        }
        const float extra = (massRatio - contactSolverConfig_.highMassRatioThreshold)
            / std::max(contactSolverConfig_.highMassRatioThreshold, 1.0f);
        return 1.0f + std::max(extra, 0.0f);
    }

void World::AdvanceDynamicBodies(float dt) {

        if (dt <= 0.0f) {
            return;
        }
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += body.velocity * dt;
        }
    }

void World::ResolveTOIImpact(const TOIEvent& hit) {

        Body& a = bodies_[hit.a];
        Body& b = bodies_[hit.b];
        const Vec3 n = Normalize(hit.normal);
        const Vec3 relativeVelocity = a.velocity - b.velocity;
        const float vn = Dot(relativeVelocity, n);
        if (vn < 0.0f) {
            const float invMassSum = a.invMass + b.invMass;
            if (invMassSum > kEpsilon) {
                const float speedIntoContact = -vn;
                const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);
                const float impulse = -(1.0f + restitution) * vn / invMassSum;
                const Vec3 impulseVector = n * impulse;
                if (a.invMass > 0.0f) {
                    a.velocity += impulseVector * a.invMass;
                }
                if (b.invMass > 0.0f) {
                    b.velocity -= impulseVector * b.invMass;
                }
            }
        }

        const float correctionSlop = 1e-4f;
        const float push = 4e-4f;
        const float invMassSum = a.invMass + b.invMass;
        if (invMassSum > kEpsilon) {
            const Vec3 correction = n * std::max(push, correctionSlop);
            if (a.invMass > 0.0f) {
                a.position += correction * (a.invMass / invMassSum);
            }
            if (b.invMass > 0.0f) {
                b.position -= correction * (b.invMass / invMassSum);
            }
        }
    }

World::TOIEvent World::SweepSpherePlane(std::uint32_t sphereId, std::uint32_t planeId, float maxDt) const {

        const Body& s = bodies_[sphereId];
        const Body& p = bodies_[planeId];
        Vec3 n{};
        if (!TryGetPlaneNormal(p, n)) {
            return {};
        }
        const float d0 = Dot(n, s.position) - p.planeOffset - s.radius;
        const float vn = Dot(n, s.velocity);
        if (d0 <= 0.0f || vn >= -kEpsilon) {
            return {};
        }
        const float toi = -d0 / vn;
        if (toi < 0.0f || toi > maxDt) {
            return {};
        }
        TOIEvent hit;
        hit.hit = true;
        hit.toi = toi;
        hit.a = sphereId;
        hit.b = planeId;
        hit.normal = n;
        hit.point = (s.position + s.velocity * toi) - n * s.radius;
        return hit;
    }

World::TOIEvent World::SweepSphereBox(std::uint32_t sphereId, std::uint32_t boxId, float maxDt) const {

        const Body& s = bodies_[sphereId];
        const Body& b = bodies_[boxId];
        const Quat invQ = Conjugate(Normalize(b.orientation));
        const Vec3 p0 = Rotate(invQ, s.position - b.position);
        const Vec3 vRel = Rotate(invQ, s.velocity - b.velocity);
        const Vec3 ext = b.halfExtents + Vec3{s.radius, s.radius, s.radius};

        float tEnter = 0.0f;
        float tExit = maxDt;
        int enteringAxis = -1;
        float enteringSign = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            const float p = (axis == 0) ? p0.x : (axis == 1) ? p0.y : p0.z;
            const float v = (axis == 0) ? vRel.x : (axis == 1) ? vRel.y : vRel.z;
            const float e = (axis == 0) ? ext.x : (axis == 1) ? ext.y : ext.z;

            if (std::abs(v) <= kEpsilon) {
                if (p < -e || p > e) {
                    return {};
                }
                continue;
            }

            float t1 = (-e - p) / v;
            float t2 = (e - p) / v;
            float sign = -1.0f;
            if (t1 > t2) {
                std::swap(t1, t2);
                sign = 1.0f;
            }
            if (t1 > tEnter) {
                tEnter = t1;
                enteringAxis = axis;
                enteringSign = sign;
            }
            tExit = std::min(tExit, t2);
            if (tEnter > tExit) {
                return {};
            }
        }

        if (tEnter < 0.0f || tEnter > maxDt || enteringAxis < 0) {
            return {};
        }

        Vec3 normalLocal{0.0f, 0.0f, 0.0f};
        if (enteringAxis == 0) {
            normalLocal.x = enteringSign;
        } else if (enteringAxis == 1) {
            normalLocal.y = enteringSign;
        } else {
            normalLocal.z = enteringSign;
        }

        TOIEvent hit;
        hit.hit = true;
        hit.toi = tEnter;
        hit.a = sphereId;
        hit.b = boxId;
        hit.normal = Rotate(b.orientation, normalLocal);
        const Vec3 centerAtHit = s.position + s.velocity * tEnter;
        hit.point = centerAtHit - hit.normal * s.radius;
        return hit;
    }

World::TOIEvent World::SweepSphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId, float maxDt) const {

        const Body& s = bodies_[sphereId];
        const Body& c = bodies_[capsuleId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 segA = c.position - axis * c.halfHeight;
        const Vec3 segB = c.position + axis * c.halfHeight;
        const Vec3 vRel = s.velocity - c.velocity;
        const float combinedRadius = s.radius + c.radius;

        float t = 0.0f;
        for (int i = 0; i < 12 && t <= maxDt; ++i) {
            const Vec3 center = s.position + vRel * t;
            const float segT = ClosestPointParameter(segA, segB, center);
            const Vec3 closest = segA + (segB - segA) * segT;
            Vec3 delta = center - closest;
            float dist = Length(delta);
            if (dist <= combinedRadius + 1e-4f) {
                TOIEvent hit;
                hit.hit = true;
                hit.toi = std::clamp(t, 0.0f, maxDt);
                if (dist <= kEpsilon) {
                    delta = StableDirection(vRel, {{axis, -axis, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}});
                    dist = 1.0f;
                }
                hit.normal = delta * (1.0f / dist);
                hit.a = sphereId;
                hit.b = capsuleId;
                hit.point = center - hit.normal * s.radius;
                return hit;
            }

            const Vec3 n = delta / std::max(dist, kEpsilon);
            const float closingSpeed = Dot(vRel, n);
            if (closingSpeed >= -kEpsilon) {
                break;
            }
            const float dt = (dist - combinedRadius) / -closingSpeed;
            if (dt <= 1e-5f) {
                t += 1e-5f;
            } else {
                t += dt;
            }
        }
        return {};
    }

World::TOIEvent World::FindEarliestTOI(float maxDt) const {

        TOIEvent earliest;
        earliest.toi = maxDt + 1.0f;

        for (std::uint32_t i = 0; i < bodies_.size(); ++i) {
            const Body& a = bodies_[i];
            if (a.shape != ShapeType::Sphere || a.invMass == 0.0f || a.isSleeping) {
                continue;
            }
            for (std::uint32_t j = 0; j < bodies_.size(); ++j) {
                if (i == j) {
                    continue;
                }
                const Body& b = bodies_[j];
                TOIEvent hit;
                if (b.shape == ShapeType::Plane) {
                    hit = SweepSpherePlane(i, j, maxDt);
                } else if (b.shape == ShapeType::Box) {
                    hit = SweepSphereBox(i, j, maxDt);
                } else if (b.shape == ShapeType::Capsule) {
                    hit = SweepSphereCapsule(i, j, maxDt);
                }
                if (!hit.hit) {
                    continue;
                }
                if (!earliest.hit || hit.toi < earliest.toi - 1e-6f || (std::abs(hit.toi - earliest.toi) <= 1e-6f && ((hit.a < earliest.a) || (hit.a == earliest.a && hit.b < earliest.b)))) {
                    earliest = hit;
                }
            }
        }

        return earliest.hit ? earliest : TOIEvent{};
    }

std::pair<std::uint32_t, std::uint32_t> World::CanonicalBodyOrder(std::uint32_t a, std::uint32_t b) {

        return {std::min(a, b), std::max(a, b)};
    }

std::uint64_t World::CanonicalFeaturePairId(
        std::uint32_t a,
        std::uint32_t b,
        std::uint32_t featureOnA,
        std::uint32_t featureOnB,
        std::uint16_t detail) {

        const bool canonicalOrder = a <= b;
        const std::uint32_t loFeature = canonicalOrder ? featureOnA : featureOnB;
        const std::uint32_t hiFeature = canonicalOrder ? featureOnB : featureOnA;
        return (static_cast<std::uint64_t>(loFeature) << 32u)
             | (static_cast<std::uint64_t>(hiFeature) << 16u)
             | static_cast<std::uint64_t>(detail);
    }

ContactKey World::MakeContactKey(
        std::uint32_t a,
        std::uint32_t b,
        std::uint8_t manifoldType,
        std::uint64_t canonicalFeatureId) {

        const auto [lo, hi] = CanonicalBodyOrder(a, b);
        return ContactKey{lo, hi, manifoldType, canonicalFeatureId};
    }

std::uint64_t World::ContactKeyStableValue(const ContactKey& key) {

        return (static_cast<std::uint64_t>(key.manifoldType) << 56u)
             ^ (static_cast<std::uint64_t>(key.loBody) << 28u)
             ^ static_cast<std::uint64_t>(key.hiBody)
             ^ key.canonicalFeatureId;
    }

ManifoldKey World::MakeManifoldId(std::uint32_t a, std::uint32_t b, std::uint8_t manifoldType) {

        const auto [lo, hi] = CanonicalBodyOrder(a, b);
        return ManifoldKey{lo, hi, manifoldType};
    }

PersistentPointKey World::MakePersistentPointKey(const ManifoldKey& manifoldId, std::uint64_t canonicalFeatureId, std::uint8_t ordinal) {

        return PersistentPointKey{manifoldId, canonicalFeatureId, ordinal};
    }

void World::WakeBody(Body& body) {

        if (body.invMass == 0.0f) {
            return;
        }
        body.isSleeping = false;
        body.sleepCounter = 0;
    }

void World::WakeConnectedBodies(std::uint32_t start) {

        if (start >= bodies_.size()) {
            return;
        }
        if (bodies_[start].invMass == 0.0f) {
            return;
        }

        std::vector<bool> visited(bodies_.size(), false);
        std::vector<std::uint32_t> stack{start};
        visited[start] = true;
        while (!stack.empty()) {
            const std::uint32_t id = stack.back();
            stack.pop_back();
            WakeBody(bodies_[id]);

            for (const Manifold& m : manifolds_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (m.a == id) other = m.b;
                else if (m.b == id) other = m.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const DistanceJoint& j : joints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const HingeJoint& j : hingeJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const BallSocketJoint& j : ballSocketJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const FixedJoint& j : fixedJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const PrismaticJoint& j : prismaticJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const ServoJoint& j : servoJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }
        }
    }

float World::ProjectBoxOntoAxis(const Body& box, const Vec3& axis) {

        const Vec3 u0 = Rotate(box.orientation, {1.0f, 0.0f, 0.0f});
        const Vec3 u1 = Rotate(box.orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 u2 = Rotate(box.orientation, {0.0f, 0.0f, 1.0f});
        return std::abs(Dot(axis, u0)) * box.halfExtents.x
             + std::abs(Dot(axis, u1)) * box.halfExtents.y
             + std::abs(Dot(axis, u2)) * box.halfExtents.z;
    }

Vec3 World::ClosestPointOnBox(const Body& box, const Vec3& worldPoint) {

        const Quat invQ = Conjugate(Normalize(box.orientation));
        const Vec3 local = Rotate(invQ, worldPoint - box.position);
        const Vec3 clamped = {
            std::clamp(local.x, -box.halfExtents.x, box.halfExtents.x),
            std::clamp(local.y, -box.halfExtents.y, box.halfExtents.y),
            std::clamp(local.z, -box.halfExtents.z, box.halfExtents.z),
        };
        return box.position + Rotate(box.orientation, clamped);
    }

void World::IntegrateForces(float dt) {

        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            const Vec3 linearAcceleration = gravity_ + body.force * body.invMass;
            const Vec3 angularAcceleration = body.InvInertiaWorld() * body.torque;

            body.velocity += linearAcceleration * dt;
            body.angularVelocity += angularAcceleration * dt;
        }
    }

void World::IntegrateVelocities(float dt) {

        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += body.velocity * dt;
        }
    }

void World::IntegrateOrientation(float dt) {

        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            const Quat omega{0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z};
            const Quat dq = (omega * body.orientation) * (0.5f * dt);
            body.orientation = Normalize(body.orientation + dq);
            AssertQuaternionNormalized(body.orientation);
        }
    }

void World::AddContact(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& normal,
        const Vec3& point,
        float penetration,
        std::uint8_t manifoldType,
        std::uint64_t canonicalFeatureId) {

        Contact c;
        c.a = a;
        c.b = b;
        c.normal = normal;
        c.point = point;
        c.penetration = std::max(penetration, 0.0f);
        c.manifoldType = manifoldType;
        c.featureKey = canonicalFeatureId;
        c.key = ContactKeyStableValue(MakeContactKey(a, b, manifoldType, canonicalFeatureId));
        assert(IsFinite(c.normal));
        assert(IsFinite(c.point));
        assert(IsFinite(c.penetration));
        assert(c.penetration >= 0.0f);

        contacts_.push_back(c);
        const Body& bodyA = bodies_[a];
        const Body& bodyB = bodies_[b];
        const float relSpeed = Length(bodyB.velocity - bodyA.velocity);
        const bool contactTouchesSleepingBody = (bodyA.isSleeping && !bodyB.isSleeping)
                                             || (!bodyA.isSleeping && bodyB.isSleeping);
        const bool strongContact = relSpeed >= kWakeContactRelativeSpeedThreshold
                                || c.penetration >= kWakeContactPenetrationThreshold
                                || contactTouchesSleepingBody;
        if (strongContact) {
            WakeConnectedBodies(a);
            WakeConnectedBodies(b);
        }
    }

int World::FindBlockSlot(const Manifold& manifold, std::uint64_t contactKey) {

        for (int slot = 0; slot < 2; ++slot) {
            if (manifold.blockSlotValid[slot] && manifold.blockContactKeys[slot] == contactKey) {
                return slot;
            }
        }
        return -1;
    }

std::array<float, 3>* World::FindPerContactImpulseCache(Manifold& manifold, std::uint64_t contactKey) {

        auto it = manifold.cachedImpulseByContactKey.find(contactKey);
        if (it == manifold.cachedImpulseByContactKey.end()) {
            return nullptr;
        }
        return &it->second;
    }

const std::array<float, 3>* World::FindPerContactImpulseCache(const Manifold& manifold, std::uint64_t contactKey) {

        auto it = manifold.cachedImpulseByContactKey.find(contactKey);
        if (it == manifold.cachedImpulseByContactKey.end()) {
            return nullptr;
        }
        return &it->second;
    }

std::array<float, 3>& World::EnsurePerContactImpulseCache(Manifold& manifold, std::uint64_t contactKey) {

        return manifold.cachedImpulseByContactKey[contactKey];
    }

int World::FindFirstFreeBlockSlot(const std::array<bool, 2>& slotOccupied) {

        for (int slot = 0; slot < 2; ++slot) {
            if (!slotOccupied[slot]) {
                return slot;
            }
        }
        return -1;
    }

int World::EnsureBlockSlotForContact(
        Manifold& manifold,
        const Contact& contact,
        std::array<bool, 2>& slotOccupied) {

        int slot = FindBlockSlot(manifold, contact.key);
        if (slot >= 0) {
            slotOccupied[slot] = true;
            std::array<float, 3>& entry = EnsurePerContactImpulseCache(manifold, contact.key);
            entry[0] = std::max(contact.normalImpulseSum, 0.0f);
            entry[1] = contact.tangentImpulseSum0;
            entry[2] = contact.tangentImpulseSum1;
            return slot;
        }

        slot = FindFirstFreeBlockSlot(slotOccupied);
        if (slot < 0) {
            std::array<float, 3>& entry = EnsurePerContactImpulseCache(manifold, contact.key);
            entry[0] = std::max(contact.normalImpulseSum, 0.0f);
            entry[1] = contact.tangentImpulseSum0;
            entry[2] = contact.tangentImpulseSum1;
            return -1;
        }

        manifold.blockSlotValid[slot] = true;
        manifold.blockContactKeys[slot] = contact.key;
        const std::array<float, 3>& entry = EnsurePerContactImpulseCache(manifold, contact.key);
        manifold.blockNormalImpulseSum[slot] = std::max(entry[0], 0.0f);
        slotOccupied[slot] = true;
        return slot;
    }

void World::RefreshManifoldBlockCache(Manifold& manifold) {

        std::unordered_set<std::uint64_t> currentKeys;
        currentKeys.reserve(manifold.contacts.size());
        std::array<bool, 2> slotOccupied{false, false};
        std::array<int, 2> slotToContactIndex{-1, -1};
        std::vector<std::size_t> contactVisitOrder;
        contactVisitOrder.reserve(manifold.contacts.size());
        for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
            contactVisitOrder.push_back(i);
        }
        if (manifold.contacts.size() >= 2
            && manifold.selectedBlockContactKeys[0] != 0u
            && manifold.selectedBlockContactKeys[1] != 0u) {
            std::stable_sort(
                contactVisitOrder.begin(),
                contactVisitOrder.end(),
                [&](std::size_t lhs, std::size_t rhs) {
                    const bool lhsSelected = manifold.contacts[lhs].key == manifold.selectedBlockContactKeys[0]
                        || manifold.contacts[lhs].key == manifold.selectedBlockContactKeys[1];
                    const bool rhsSelected = manifold.contacts[rhs].key == manifold.selectedBlockContactKeys[0]
                        || manifold.contacts[rhs].key == manifold.selectedBlockContactKeys[1];
                    if (lhsSelected != rhsSelected) {
                        return lhsSelected;
                    }
                    return lhs < rhs;
                });
        }

        for (const std::size_t i : contactVisitOrder) {
            Contact& contact = manifold.contacts[i];
            currentKeys.insert(contact.key);
            std::array<float, 3>& entry = EnsurePerContactImpulseCache(manifold, contact.key);
            entry[0] = std::max(entry[0], 0.0f);
            contact.normalImpulseSum = std::max(entry[0], 0.0f);
            contact.tangentImpulseSum0 = entry[1];
            contact.tangentImpulseSum1 = entry[2];
            contact.tangentImpulseSum = contact.tangentImpulseSum0;
            const int slot = EnsureBlockSlotForContact(manifold, contact, slotOccupied);
            if (slot >= 0 && slotToContactIndex[slot] < 0) {
                slotToContactIndex[slot] = static_cast<int>(i);
            }
        }

        for (auto it = manifold.cachedImpulseByContactKey.begin(); it != manifold.cachedImpulseByContactKey.end();) {
            if (currentKeys.find(it->first) == currentKeys.end()) {
                it = manifold.cachedImpulseByContactKey.erase(it);
            } else {
                ++it;
            }
        }

        for (int slot = 0; slot < 2; ++slot) {
            if (!slotOccupied[slot]) {
                manifold.blockSlotValid[slot] = false;
                manifold.blockContactKeys[slot] = 0u;
                manifold.blockNormalImpulseSum[slot] = 0.0f;
                continue;
            }
            if (slotToContactIndex[slot] < 0) {
                continue;
            }
            manifold.contacts[slotToContactIndex[slot]].normalImpulseSum = manifold.blockNormalImpulseSum[slot];
            std::array<float, 3>& entry = EnsurePerContactImpulseCache(
                manifold, manifold.contacts[slotToContactIndex[slot]].key);
            entry[0] = manifold.blockNormalImpulseSum[slot];
        }

        if (manifold.contacts.size() == 2 && slotToContactIndex[0] == 1 && slotToContactIndex[1] == 0) {
            std::swap(manifold.contacts[0], manifold.contacts[1]);
        }
    }

std::uint64_t World::StableContactFallbackKey(const Contact& contact) {

        std::uint64_t h = contact.key;
        h ^= static_cast<std::uint64_t>(contact.a) << 32;
        h ^= static_cast<std::uint64_t>(contact.b);
        const auto quantize = [](float value) {
            return static_cast<std::int32_t>(std::lround(value * 10000.0f));
        };
        h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(quantize(contact.normal.x))) << 1;
        h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(quantize(contact.normal.y))) << 11;
        h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(quantize(contact.normal.z))) << 21;
        return h;
    }

bool World::ContactComesBefore(const Contact& lhs, const Contact& rhs) {

        if (lhs.key != rhs.key) {
            return lhs.key < rhs.key;
        }
        if (lhs.a != rhs.a) {
            return lhs.a < rhs.a;
        }
        if (lhs.b != rhs.b) {
            return lhs.b < rhs.b;
        }
        if (lhs.penetration != rhs.penetration) {
            return lhs.penetration > rhs.penetration;
        }
        if (lhs.normal.x != rhs.normal.x) {
            return lhs.normal.x < rhs.normal.x;
        }
        if (lhs.normal.y != rhs.normal.y) {
            return lhs.normal.y < rhs.normal.y;
        }
        if (lhs.normal.z != rhs.normal.z) {
            return lhs.normal.z < rhs.normal.z;
        }
        if (lhs.point.x != rhs.point.x) {
            return lhs.point.x < rhs.point.x;
        }
        if (lhs.point.y != rhs.point.y) {
            return lhs.point.y < rhs.point.y;
        }
        if (lhs.point.z != rhs.point.z) {
            return lhs.point.z < rhs.point.z;
        }
        return StableContactFallbackKey(lhs) < StableContactFallbackKey(rhs);
    }

void World::SortManifoldContacts(std::vector<Contact>& contacts) {

        std::stable_sort(contacts.begin(), contacts.end(), ContactComesBefore);
    }

World::ManifoldQualityScore World::ComputeManifoldQualityScore(const Manifold& manifold) {

        ManifoldQualityScore score{};
        if (manifold.contacts.empty()) {
            return score;
        }
        Vec3 normal = Normalize(manifold.normal);
        if (LengthSquared(normal) <= kEpsilon) {
            normal = manifold.contacts.front().normal;
        }
        normal = Normalize(normal);
        Vec3 centroid{0.0f, 0.0f, 0.0f};
        for (const Contact& c : manifold.contacts) {
            score.penetration += std::max(c.penetration, 0.0f);
            centroid += c.point;
            score.normalCoherence += std::max(0.0f, Dot(Normalize(c.normal), normal));
        }
        const float invN = 1.0f / static_cast<float>(manifold.contacts.size());
        centroid *= invN;
        score.penetration *= invN;
        score.normalCoherence *= invN;
        for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
            const Vec3 ai = manifold.contacts[i].point - centroid;
            for (std::size_t j = i + 1; j < manifold.contacts.size(); ++j) {
                const Vec3 aj = manifold.contacts[j].point - centroid;
                score.spreadArea = std::max(score.spreadArea, Length(Cross(ai, aj)));
            }
        }
        score.total = score.penetration + (0.5f * score.spreadArea) + (0.25f * score.normalCoherence);
        return score;
    }

void World::ReduceManifoldToMaxPoints(Manifold& manifold, std::size_t maxPoints) {

        if (manifold.contacts.size() <= maxPoints) {
            return;
        }
        const Vec3 centroid = [&]() {
            Vec3 c{0.0f, 0.0f, 0.0f};
            for (const Contact& contact : manifold.contacts) {
                c += contact.point;
            }
            return c * (1.0f / static_cast<float>(manifold.contacts.size()));
        }();
        Vec3 manifoldNormal = Normalize(manifold.normal);
        if (LengthSquared(manifoldNormal) <= kEpsilon) {
            manifoldNormal = {0.0f, 1.0f, 0.0f};
        }
        std::stable_sort(manifold.contacts.begin(), manifold.contacts.end(), [&](const Contact& lhs, const Contact& rhs) {
            const float lhsSpread = LengthSquared(lhs.point - centroid);
            const float rhsSpread = LengthSquared(rhs.point - centroid);
            const float lhsScore = std::max(lhs.penetration, 0.0f) + (0.25f * lhsSpread) + 0.25f * std::max(0.0f, Dot(lhs.normal, manifoldNormal));
            const float rhsScore = std::max(rhs.penetration, 0.0f) + (0.25f * rhsSpread) + 0.25f * std::max(0.0f, Dot(rhs.normal, manifoldNormal));
            if (lhsScore != rhsScore) {
                return lhsScore > rhsScore;
            }
            return ContactComesBefore(lhs, rhs);
        });
        manifold.contacts.resize(maxPoints);
    }

void World::ManifoldManager::Process(Manifold& manifold, const Manifold* previous) const {

            const std::vector<Contact> originalContacts = manifold.contacts;
            std::unordered_map<std::uint64_t, Contact> mergedByKey;
            mergedByKey.reserve(manifold.contacts.size());
            for (const Contact& contact : manifold.contacts) {
                if (!std::isfinite(contact.penetration) || contact.penetration <= 0.0f) {
                    continue;
                }
                auto it = mergedByKey.find(contact.key);
                if (it == mergedByKey.end()) {
                    mergedByKey.emplace(contact.key, contact);
                    continue;
                }
                Contact& existing = it->second;
                if (contact.persistenceAge > existing.persistenceAge
                    || (contact.persistenceAge == existing.persistenceAge && contact.penetration > existing.penetration)) {
                    existing = contact;
                }
            }
            manifold.contacts.clear();
            manifold.contacts.reserve(mergedByKey.size());
            for (const auto& [_, contact] : mergedByKey) {
                manifold.contacts.push_back(contact);
            }
            ReduceManifoldToMaxPoints(manifold, 4);
            SortManifoldContacts(manifold.contacts);
            const ManifoldQualityScore qualityScore = ComputeManifoldQualityScore(manifold);
            manifold.lowQuality = qualityScore.total < 0.05f || qualityScore.normalCoherence < 0.5f;
#ifndef NDEBUG
            if (qualityScore.total < 0.05f) {
                ++world_.solverTelemetry_.manifoldQualityLow;
            } else if (qualityScore.total < 0.2f) {
                ++world_.solverTelemetry_.manifoldQualityMedium;
            } else {
                ++world_.solverTelemetry_.manifoldQualityHigh;
            }
            if (originalContacts.size() > manifold.contacts.size()) {
                world_.solverTelemetry_.manifoldPointRemoves += (originalContacts.size() - manifold.contacts.size());
            }
            if (previous != nullptr) {
                std::unordered_map<std::uint64_t, int> oldCounts;
                std::unordered_map<std::uint64_t, int> newCounts;
                for (const Contact& c : previous->contacts) {
                    ++oldCounts[c.key];
                }
                for (const Contact& c : manifold.contacts) {
                    ++newCounts[c.key];
                }
                for (const auto& [key, oldCount] : oldCounts) {
                    const int newCount = newCounts.count(key) > 0 ? newCounts.at(key) : 0;
                    if (newCount < oldCount) {
                        world_.solverTelemetry_.manifoldPointRemoves += static_cast<std::uint64_t>(oldCount - newCount);
                    }
                }
                for (const auto& [key, newCount] : newCounts) {
                    const int oldCount = oldCounts.count(key) > 0 ? oldCounts.at(key) : 0;
                    if (newCount > oldCount) {
                        world_.solverTelemetry_.manifoldPointAdds += static_cast<std::uint64_t>(newCount - oldCount);
                    }
                }
                if (previous->contacts.size() == manifold.contacts.size()) {
                    bool orderChanged = false;
                    for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
                        if (previous->contacts[i].key != manifold.contacts[i].key) {
                            orderChanged = true;
                            break;
                        }
                    }
                    if (orderChanged) {
                        ++world_.solverTelemetry_.manifoldPointReorders;
                    }
                }
            }
#endif
        }

void World::ManageManifoldContacts(Manifold& manifold, const Manifold* previous) {

        ManifoldManager manager(*this);
        manager.Process(manifold, previous);
    }

bool World::PersistentPointCandidateLess(
        const PersistentPointMatchCandidate& lhs,
        const PersistentPointMatchCandidate& rhs) {

        const float lhsScore = lhs.localAnchorDriftSq + lhs.worldAnchorDriftSq;
        const float rhsScore = rhs.localAnchorDriftSq + rhs.worldAnchorDriftSq;
        if (lhsScore != rhsScore) {
            return lhsScore < rhsScore;
        }
        if (lhs.localAnchorDriftSq != rhs.localAnchorDriftSq) {
            return lhs.localAnchorDriftSq < rhs.localAnchorDriftSq;
        }
        if (lhs.worldAnchorDriftSq != rhs.worldAnchorDriftSq) {
            return lhs.worldAnchorDriftSq < rhs.worldAnchorDriftSq;
        }
        if (lhs.key.ordinal != rhs.key.ordinal) {
            return lhs.key.ordinal < rhs.key.ordinal;
        }
        return PersistentPointKeyHash{}(lhs.key) < PersistentPointKeyHash{}(rhs.key);
    }

bool World::TryMatchPersistentPoint(
        const std::unordered_map<PersistentPointKey, PersistentPointImpulseState, PersistentPointKeyHash>& previousState,
        const ManifoldKey& manifoldId,
        const Contact& contact,
        const std::unordered_set<PersistentPointKey, PersistentPointKeyHash>& usedKeys,
        PersistentPointMatchCandidate& outCandidate) {

        bool found = false;
        PersistentPointMatchCandidate bestCandidate{};
        const float localThresholdSq = kPersistenceLocalAnchorDriftThreshold * kPersistenceLocalAnchorDriftThreshold;
        const float worldThresholdSq = kPersistenceWorldAnchorDriftThreshold * kPersistenceWorldAnchorDriftThreshold;
        for (const auto& [key, state] : previousState) {
            if (!(key.manifold == manifoldId) || key.canonicalFeatureId != contact.featureKey) {
                continue;
            }
            if (usedKeys.find(key) != usedKeys.end()) {
                continue;
            }

            float localDriftSq = worldThresholdSq;
            float worldDriftSq = worldThresholdSq;
            if (state.anchorsValid && contact.anchorsValid) {
                const Vec3 dA = contact.localAnchorA - state.localAnchorA;
                const Vec3 dB = contact.localAnchorB - state.localAnchorB;
                localDriftSq = Dot(dA, dA) + Dot(dB, dB);
            }
            const Vec3 dWorld = contact.point - state.worldPoint;
            worldDriftSq = Dot(dWorld, dWorld);

            const bool localPass = !(state.anchorsValid && contact.anchorsValid) || localDriftSq <= localThresholdSq;
            const bool worldPass = worldDriftSq <= worldThresholdSq;
            if (!localPass || !worldPass) {
                continue;
            }

            PersistentPointMatchCandidate candidate;
            candidate.key = key;
            candidate.state = state;
            candidate.localAnchorDriftSq = localDriftSq;
            candidate.worldAnchorDriftSq = worldDriftSq;
            if (!found || PersistentPointCandidateLess(candidate, bestCandidate)) {
                found = true;
                bestCandidate = candidate;
            }
        }
        if (found) {
            outCandidate = bestCandidate;
        }
        return found;
    }

void World::CapturePersistentPointImpulseState(const std::vector<Manifold>& manifolds) {

        const auto previousState = persistentPointImpulses_;
        persistentPointImpulses_.clear();
        std::unordered_set<PersistentPointKey, PersistentPointKeyHash> usedKeys;
        std::uint64_t matchedPoints = 0;
        std::uint64_t newPoints = 0;
        for (const Manifold& manifold : manifolds) {
            const ManifoldKey manifoldId = MakeManifoldId(manifold.a, manifold.b, manifold.manifoldType);
            for (const Contact& contact : manifold.contacts) {
                PersistentPointKey pointKey = MakePersistentPointKey(manifoldId, contact.featureKey, 0u);
                std::uint16_t persistenceAge = 1;
                PersistentPointMatchCandidate match{};
                if (TryMatchPersistentPoint(previousState, manifoldId, contact, usedKeys, match)) {
                    pointKey = match.key;
                    persistenceAge = static_cast<std::uint16_t>(
                        std::min<std::uint32_t>(static_cast<std::uint32_t>(match.state.persistenceAge) + 1u,
                                                static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())));
                    usedKeys.insert(match.key);
                    ++matchedPoints;
                } else {
                    std::uint8_t maxOrdinal = 0u;
                    for (const auto& [key, _] : previousState) {
                        if (key.manifold == manifoldId && key.canonicalFeatureId == contact.featureKey) {
                            maxOrdinal = static_cast<std::uint8_t>(std::max<std::uint32_t>(maxOrdinal, static_cast<std::uint32_t>(key.ordinal + 1u)));
                        }
                    }
                    pointKey.ordinal = maxOrdinal;
                    ++newPoints;
                }
                persistentPointImpulses_[pointKey] = {
                    contact.normalImpulseSum,
                    contact.tangentImpulseSum0,
                    contact.tangentImpulseSum1,
                    persistenceAge,
                    contact.anchorsValid,
                    contact.localAnchorA,
                    contact.localAnchorB,
                    contact.point};
            }
        }
        persistenceMatchDiagnostics_.matchedPoints = matchedPoints;
        persistenceMatchDiagnostics_.newPoints = newPoints;
        persistenceMatchDiagnostics_.droppedPoints = previousState.size() >= usedKeys.size()
            ? static_cast<std::uint64_t>(previousState.size() - usedKeys.size())
            : 0u;
        const float denom = static_cast<float>(std::max<std::uint64_t>(1u, previousState.size()));
        persistenceMatchDiagnostics_.churnRatio = static_cast<float>(
            static_cast<double>(persistenceMatchDiagnostics_.droppedPoints + persistenceMatchDiagnostics_.newPoints) / denom);
    }

void World::BuildIslands() {

        islands_.clear();
        std::vector<bool> visited(bodies_.size(), false);

        for (std::uint32_t start = 0; start < bodies_.size(); ++start) {
            if (visited[start]) {
                continue;
            }
            if (bodies_[start].invMass == 0.0f || bodies_[start].isSleeping) {
                visited[start] = true;
                continue;
            }

            Island island;
            std::vector<std::uint32_t> stack{start};
            visited[start] = true;

            while (!stack.empty()) {
                const std::uint32_t bodyId = stack.back();
                stack.pop_back();
                island.bodies.push_back(bodyId);

                for (std::size_t mi = 0; mi < manifolds_.size(); ++mi) {
                    const Manifold& m = manifolds_[mi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (m.a == bodyId) other = m.b;
                    else if (m.b == bodyId) other = m.a;
                    else continue;

                    if (std::find(island.manifolds.begin(), island.manifolds.end(), mi) == island.manifolds.end()) {
                        island.manifolds.push_back(mi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t ji = 0; ji < joints_.size(); ++ji) {
                    const DistanceJoint& j = joints_[ji];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.joints.begin(), island.joints.end(), ji) == island.joints.end()) {
                        island.joints.push_back(ji);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t hi = 0; hi < hingeJoints_.size(); ++hi) {
                    const HingeJoint& j = hingeJoints_[hi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.hinges.begin(), island.hinges.end(), hi) == island.hinges.end()) {
                        island.hinges.push_back(hi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t bi = 0; bi < ballSocketJoints_.size(); ++bi) {
                    const BallSocketJoint& j = ballSocketJoints_[bi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.ballSockets.begin(), island.ballSockets.end(), bi) == island.ballSockets.end()) {
                        island.ballSockets.push_back(bi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t fi = 0; fi < fixedJoints_.size(); ++fi) {
                    const FixedJoint& j = fixedJoints_[fi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.fixeds.begin(), island.fixeds.end(), fi) == island.fixeds.end()) {
                        island.fixeds.push_back(fi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t pi = 0; pi < prismaticJoints_.size(); ++pi) {
                    const PrismaticJoint& j = prismaticJoints_[pi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.prismatics.begin(), island.prismatics.end(), pi) == island.prismatics.end()) {
                        island.prismatics.push_back(pi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t si = 0; si < servoJoints_.size(); ++si) {
                    const ServoJoint& j = servoJoints_[si];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.servos.begin(), island.servos.end(), si) == island.servos.end()) {
                        island.servos.push_back(si);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }
            }

            if (!island.bodies.empty()) {
                islands_.push_back(island);
            }
        }
    }

std::vector<Pair> World::ComputePotentialPairsBruteForce() const {

        std::vector<Pair> pairs;
        if (bodies_.size() < 2) {
            return pairs;
        }
        pairs.reserve((bodies_.size() * (bodies_.size() - 1)) / 2);
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
                pairs.push_back({i, j});
            }
        }
        return pairs;
    }

std::unordered_set<std::uint64_t> World::PairSetFromPairs(const std::vector<Pair>& pairs) {

        std::unordered_set<std::uint64_t> pairSet;
        pairSet.reserve(pairs.size());
        for (const Pair& pair : pairs) {
            pairSet.insert((static_cast<std::uint64_t>(pair.a) << 32) | pair.b);
        }
        return pairSet;
    }

bool World::IsConvexShape(ShapeType shape) {

        return shape == ShapeType::Sphere || shape == ShapeType::Box || shape == ShapeType::Capsule;
    }

Vec3 World::ClampPointToExtents(const Vec3& p, const Vec3& extents) {

        return {
            std::clamp(p.x, -extents.x, extents.x),
            std::clamp(p.y, -extents.y, extents.y),
            std::clamp(p.z, -extents.z, extents.z),
        };
    }

Vec3 World::StableDirection(const Vec3& primary, const std::array<Vec3, 4>& fallbacks) {

        if (LengthSquared(primary) > kEpsilon * kEpsilon) {
            return Normalize(primary);
        }
        for (const Vec3& candidate : fallbacks) {
            if (LengthSquared(candidate) > kEpsilon * kEpsilon) {
                return Normalize(candidate);
            }
        }
        return {0.0f, 1.0f, 0.0f};
    }

std::pair<float, float> World::ClosestSegmentParameters(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {

        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const Vec3 r = p1 - p2;
        const float aLen = Dot(d1, d1);
        const float eLen = Dot(d2, d2);
        const float f = Dot(d2, r);

        float s = 0.0f;
        float t = 0.0f;
        if (aLen <= kEpsilon && eLen <= kEpsilon) {
            return {0.0f, 0.0f};
        }
        if (aLen <= kEpsilon) {
            t = std::clamp(f / eLen, 0.0f, 1.0f);
            return {0.0f, t};
        }

        const float cTerm = Dot(d1, r);
        if (eLen <= kEpsilon) {
            s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
            return {s, 0.0f};
        }

        const float bDot = Dot(d1, d2);
        const float denom = aLen * eLen - bDot * bDot;
        if (std::abs(denom) > kEpsilon) {
            s = std::clamp((bDot * f - cTerm * eLen) / denom, 0.0f, 1.0f);
        } else {
            s = 0.0f;
        }

        t = (bDot * s + f) / eLen;
        if (t < 0.0f) {
            t = 0.0f;
            s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
        } else if (t > 1.0f) {
            t = 1.0f;
            s = std::clamp((bDot - cTerm) / aLen, 0.0f, 1.0f);
        }

        return {s, t};
    }

World::SegmentBoxClosest World::ClosestSegmentPointToBox(const Vec3& segA, const Vec3& segB, const Vec3& extents) {

        const Vec3 d = segB - segA;
        auto eval = [&](float t) {
            SegmentBoxClosest result;
            result.t = std::clamp(t, 0.0f, 1.0f);
            result.segmentPoint = segA + d * result.t;
            result.boxPoint = ClampPointToExtents(result.segmentPoint, extents);
            result.distSq = LengthSquared(result.segmentPoint - result.boxPoint);
            return result;
        };

        SegmentBoxClosest best = eval(0.0f);
        auto consider = [&](float t) {
            const SegmentBoxClosest candidate = eval(t);
            if (candidate.distSq < best.distSq) {
                best = candidate;
            }
        };

        consider(1.0f);

        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            const float p0 = (axisIndex == 0) ? segA.x : (axisIndex == 1 ? segA.y : segA.z);
            const float vd = (axisIndex == 0) ? d.x : (axisIndex == 1 ? d.y : d.z);
            const float extent = (axisIndex == 0) ? extents.x : (axisIndex == 1 ? extents.y : extents.z);
            if (std::abs(vd) <= kEpsilon) {
                continue;
            }
            consider((-extent - p0) / vd);
            consider(( extent - p0) / vd);
        }

        float lo = 0.0f;
        float hi = 1.0f;
        for (int i = 0; i < 32; ++i) {
            const float m1 = lo + (hi - lo) / 3.0f;
            const float m2 = hi - (hi - lo) / 3.0f;
            if (eval(m1).distSq < eval(m2).distSq) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        consider(0.5f * (lo + hi));

        return best;
    }

void World::SpherePlane(std::uint32_t sphereId, std::uint32_t planeId) {

        const Body& s = bodies_[sphereId];
        const Body& p = bodies_[planeId];
        Vec3 n{};
        if (!TryGetPlaneNormal(p, n)) {
            return;
        }
        const float signedDistance = Dot(n, s.position) - p.planeOffset;
        if (signedDistance >= s.radius) {
            return;
        }
        const std::uint64_t featureId = CanonicalFeaturePairId(sphereId, planeId, 0u, 0u);
        AddContact(sphereId, planeId, -n, s.position - n * s.radius, s.radius - signedDistance, 2u, featureId);
    }

void World::CapsulePlane(std::uint32_t capsuleId, std::uint32_t planeId) {

        const Body& c = bodies_[capsuleId];
        const Body& p = bodies_[planeId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 ends[2] = {c.position - axis * c.halfHeight, c.position + axis * c.halfHeight};
        Vec3 n{};
        if (!TryGetPlaneNormal(p, n)) {
            return;
        }
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const Vec3& center = ends[endpoint];
            const float signedDistance = Dot(n, center) - p.planeOffset;
            if (signedDistance < c.radius) {
                const std::uint32_t capsuleFeature = static_cast<std::uint32_t>(endpoint);
                const std::uint64_t featureId = CanonicalFeaturePairId(capsuleId, planeId, capsuleFeature, 0u);
                AddContact(capsuleId, planeId, -n, center - n * c.radius, c.radius - signedDistance, 3u, featureId);
            }
        }
    }

void World::SphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId) {

        const Body& s = bodies_[sphereId];
        const Body& c = bodies_[capsuleId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 a = c.position - axis * c.halfHeight;
        const Vec3 b = c.position + axis * c.halfHeight;
        const Vec3 ab = b - a;
        const float denom = Dot(ab, ab);
        float t = 0.0f;
        if (denom > kEpsilon) {
            t = std::clamp(Dot(s.position - a, ab) / denom, 0.0f, 1.0f);
        }
        const Vec3 closest = a + ab * t;
        const Vec3 delta = closest - s.position;
        const float distSq = LengthSquared(delta);
        const float radiusSum = s.radius + c.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }
        const float dist = std::sqrt(std::max(distSq, kEpsilon));
        const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{0.0f, 1.0f, 0.0f};
        const std::uint8_t capsuleFeature = (t <= 1e-3f) ? 0u : ((t >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureId = CanonicalFeaturePairId(sphereId, capsuleId, 0u, capsuleFeature);
        AddContact(sphereId, capsuleId, normal, s.position + normal * s.radius, radiusSum - dist, 4u, featureId);
    }

void World::CapsuleCapsule(std::uint32_t aId, std::uint32_t bId) {

        const Body& a = bodies_[aId];
        const Body& b = bodies_[bId];

        const Vec3 axisA = Normalize(Rotate(a.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 axisB = Normalize(Rotate(b.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 p1 = a.position - axisA * a.halfHeight;
        const Vec3 q1 = a.position + axisA * a.halfHeight;
        const Vec3 p2 = b.position - axisB * b.halfHeight;
        const Vec3 q2 = b.position + axisB * b.halfHeight;

        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const auto [s, t] = ClosestSegmentParameters(p1, q1, p2, q2);

        const Vec3 c1 = p1 + d1 * s;
        const Vec3 c2 = p2 + d2 * t;
        const Vec3 delta = c2 - c1;
        const float distSq = LengthSquared(delta);
        const float radiusSum = a.radius + b.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }

        const float dist = std::sqrt(std::max(distSq, 0.0f));
        const Vec3 centerDelta = b.position - a.position;
        Vec3 normal = StableDirection(delta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
        if (Dot(normal, centerDelta) < 0.0f) {
            normal = -normal;
        }

        const Vec3 point = c1 + normal * a.radius;
        const std::uint8_t featureA = (s <= 1e-3f) ? 0u : ((s >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint8_t featureB = (t <= 1e-3f) ? 0u : ((t >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, featureA, featureB);
        AddContact(aId, bId, normal, point, radiusSum - dist, 5u, featureId);

        const float parallelFactor = std::abs(Dot(axisA, axisB));
        if (parallelFactor > 0.98f && featureA == 2u && featureB == 2u) {
            auto addProjectedEndpointContact = [&](const Vec3& endpointA, std::uint8_t endpointFeature) {
                const auto [sProj, tProj] = ClosestSegmentParameters(endpointA, endpointA, p2, q2);
                (void)sProj;
                const Vec3 onB = p2 + d2 * tProj;
                const Vec3 endpointDelta = onB - endpointA;
                const float endpointDistSq = LengthSquared(endpointDelta);
                if (endpointDistSq > radiusSum * radiusSum) {
                    return;
                }
                Vec3 endpointNormal = StableDirection(endpointDelta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
                if (Dot(endpointNormal, centerDelta) < 0.0f) {
                    endpointNormal = -endpointNormal;
                }
                const float endpointDist = std::sqrt(std::max(endpointDistSq, 0.0f));
                const std::uint8_t featureBProj = (tProj <= 1e-3f) ? 0u : ((tProj >= 1.0f - 1e-3f) ? 1u : 2u);
                const std::uint64_t manifoldFeatureId = CanonicalFeaturePairId(aId, bId, endpointFeature, featureBProj, 1u);
                AddContact(aId, bId, endpointNormal, endpointA + endpointNormal * a.radius, radiusSum - endpointDist, 5u, manifoldFeatureId);
            };

            addProjectedEndpointContact(p1, 0u);
            addProjectedEndpointContact(q1, 1u);
        }
    }

void World::CapsuleBox(std::uint32_t capsuleId, std::uint32_t boxId) {

        const Body& c = bodies_[capsuleId];
        const Body& b = bodies_[boxId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 segAWorld = c.position - axis * c.halfHeight;
        const Vec3 segBWorld = c.position + axis * c.halfHeight;

        const Quat invBoxOrientation = Conjugate(Normalize(b.orientation));
        const Vec3 segA = Rotate(invBoxOrientation, segAWorld - b.position);
        const Vec3 segB = Rotate(invBoxOrientation, segBWorld - b.position);
        const SegmentBoxClosest closest = ClosestSegmentPointToBox(segA, segB, b.halfExtents);
        if (closest.distSq > c.radius * c.radius) {
            return;
        }

        const Vec3 localDelta = closest.segmentPoint - closest.boxPoint;
        const float dist = std::sqrt(std::max(closest.distSq, 0.0f));
        Vec3 normalLocal = StableDirection(localDelta, {closest.segmentPoint, segB - segA, axis, {0.0f, 1.0f, 0.0f}});
        if (closest.distSq <= kEpsilon * kEpsilon) {
            const float dx = b.halfExtents.x - std::abs(closest.segmentPoint.x);
            const float dy = b.halfExtents.y - std::abs(closest.segmentPoint.y);
            const float dz = b.halfExtents.z - std::abs(closest.segmentPoint.z);
            if (dx <= dy && dx <= dz) {
                normalLocal = {(closest.segmentPoint.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};
            } else if (dy <= dz) {
                normalLocal = {0.0f, (closest.segmentPoint.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
            } else {
                normalLocal = {0.0f, 0.0f, (closest.segmentPoint.z >= 0.0f) ? 1.0f : -1.0f};
            }
        }
        const Vec3 normalWorld = Rotate(b.orientation, normalLocal);
        const Vec3 pointWorld = b.position + Rotate(b.orientation, closest.boxPoint);
        const float segT = closest.t;
        const std::uint8_t capsuleFeature = (segT <= 1e-3f) ? 0u : ((segT >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureId = CanonicalFeaturePairId(capsuleId, boxId, capsuleFeature, 0u);
        AddContact(capsuleId, boxId, -normalWorld, pointWorld, c.radius - dist, 6u, featureId);
    }

void World::BoxPlane(std::uint32_t boxId, std::uint32_t planeId) {

        const Body& box = bodies_[boxId];
        const Body& plane = bodies_[planeId];
        Vec3 n{};
        if (!TryGetPlaneNormal(plane, n)) {
            return;
        }

        const Vec3 axes[3] = {
            Rotate(box.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(box.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(box.orientation, {0.0f, 0.0f, 1.0f}),
        };

        int added = 0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Vec3 local = {
                        sx * box.halfExtents.x,
                        sy * box.halfExtents.y,
                        sz * box.halfExtents.z,
                    };
                    const Vec3 worldPoint = box.position
                        + axes[0] * local.x
                        + axes[1] * local.y
                        + axes[2] * local.z;
                    const float signedDistance = Dot(n, worldPoint) - plane.planeOffset;
                    if (signedDistance < 0.0f) {
                        const std::uint8_t vertexId = static_cast<std::uint8_t>((sx > 0 ? 1 : 0) | ((sy > 0 ? 1 : 0) << 1) | ((sz > 0 ? 1 : 0) << 2));
                        const std::uint64_t featureId = CanonicalFeaturePairId(boxId, planeId, vertexId, 0u);
                        AddContact(boxId, planeId, -n, worldPoint, -signedDistance, 7u, featureId);
                        ++added;
                        if (added >= 4) {
                            return;
                        }
                    }
                }
            }
        }
    }

void World::SphereBox(std::uint32_t sphereId, std::uint32_t boxId) {

        const Body& s = bodies_[sphereId];
        const Body& b = bodies_[boxId];

        const Quat invBoxOrientation = Conjugate(Normalize(b.orientation));
        const Vec3 sphereCenterLocal = Rotate(invBoxOrientation, s.position - b.position);

        const Vec3 closestLocal = {
            std::clamp(sphereCenterLocal.x, -b.halfExtents.x, b.halfExtents.x),
            std::clamp(sphereCenterLocal.y, -b.halfExtents.y, b.halfExtents.y),
            std::clamp(sphereCenterLocal.z, -b.halfExtents.z, b.halfExtents.z),
        };

        const Vec3 deltaLocal = sphereCenterLocal - closestLocal;
        const float distSq = LengthSquared(deltaLocal);
        if (distSq > s.radius * s.radius) {
            return;
        }

        Vec3 normalWorld{};
        Vec3 pointWorld{};
        float penetration = 0.0f;

        if (distSq > kEpsilon) {
            const float dist = std::sqrt(distSq);
            const Vec3 normalLocal = deltaLocal / dist;
            normalWorld = Rotate(b.orientation, normalLocal);
            pointWorld = b.position + Rotate(b.orientation, closestLocal);
            penetration = s.radius - dist;
        } else {
            const float dx = b.halfExtents.x - std::abs(sphereCenterLocal.x);
            const float dy = b.halfExtents.y - std::abs(sphereCenterLocal.y);
            const float dz = b.halfExtents.z - std::abs(sphereCenterLocal.z);

            Vec3 normalLocal;
            if (dx <= dy && dx <= dz) {
                normalLocal = {(sphereCenterLocal.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};
                penetration = s.radius + dx;
            } else if (dy <= dz) {
                normalLocal = {0.0f, (sphereCenterLocal.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
                penetration = s.radius + dy;
            } else {
                normalLocal = {0.0f, 0.0f, (sphereCenterLocal.z >= 0.0f) ? 1.0f : -1.0f};
                penetration = s.radius + dz;
            }

            const Vec3 facePointLocal = {
                normalLocal.x != 0.0f ? normalLocal.x * b.halfExtents.x : sphereCenterLocal.x,
                normalLocal.y != 0.0f ? normalLocal.y * b.halfExtents.y : sphereCenterLocal.y,
                normalLocal.z != 0.0f ? normalLocal.z * b.halfExtents.z : sphereCenterLocal.z,
            };

            normalWorld = Rotate(b.orientation, normalLocal);
            pointWorld = b.position + Rotate(b.orientation, facePointLocal);
        }

        std::uint8_t referenceFace = 0;
        std::uint8_t incidentFeature = 0;
        const Vec3 localPoint = Rotate(invBoxOrientation, pointWorld - b.position);
        const float dx = std::abs(std::abs(localPoint.x) - b.halfExtents.x);
        const float dy = std::abs(std::abs(localPoint.y) - b.halfExtents.y);
        const float dz = std::abs(std::abs(localPoint.z) - b.halfExtents.z);
        if (dx <= dy && dx <= dz) {
            referenceFace = static_cast<std::uint8_t>((localPoint.x >= 0.0f) ? 0 : 1);
        } else if (dy <= dz) {
            referenceFace = static_cast<std::uint8_t>((localPoint.y >= 0.0f) ? 2 : 3);
        } else {
            referenceFace = static_cast<std::uint8_t>((localPoint.z >= 0.0f) ? 4 : 5);
        }
        incidentFeature = 0;
        const std::uint64_t featureId = CanonicalFeaturePairId(sphereId, boxId, incidentFeature, referenceFace);
        AddContact(sphereId, boxId, -normalWorld, pointWorld, penetration, 8u, featureId);
    }

void World::WarmStartJoints() {

        for (DistanceJoint& j : joints_) {
            if (j.impulseSum == 0.0f) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 delta = (b.position + rb) - (a.position + ra);
            const float len = Length(delta);
            if (len <= kEpsilon) {
                continue;
            }
            const Vec3 n = delta / len;
            ApplyImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), ra, rb, j.impulseSum * n);
        }

        for (HingeJoint& j : hingeJoints_) {
            const float linearSum = std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ);
            const float angularSum = std::abs(j.angularImpulse1) + std::abs(j.angularImpulse2) + std::abs(j.motorImpulseSum);
            if (linearSum + angularSum <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 impulse{j.impulseX, j.impulseY, j.impulseZ};
            ApplyImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), ra, rb, impulse);

            const Vec3 axisA = Normalize(Rotate(a.orientation, j.localAxisA));
            Vec3 t1 = Cross(axisA, {1.0f, 0.0f, 0.0f});
            if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axisA, {0.0f, 0.0f, 1.0f});
            t1 = Normalize(t1);
            const Vec3 t2 = Normalize(Cross(axisA, t1));
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.angularImpulse1 * t1);
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.angularImpulse2 * t2);
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.motorImpulseSum * axisA);
        }

        for (BallSocketJoint& j : ballSocketJoints_) {
            if (std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ) <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            ApplyImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), ra, rb, {j.impulseX, j.impulseY, j.impulseZ});
        }

        for (FixedJoint& j : fixedJoints_) {
            const float linearSum = std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ);
            const float angularSum = std::abs(j.angularImpulseX) + std::abs(j.angularImpulseY) + std::abs(j.angularImpulseZ);
            if (linearSum + angularSum <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Mat3 invIA = a.InvInertiaWorld();
            const Mat3 invIB = b.InvInertiaWorld();
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            ApplyImpulse(a, b, invIA, invIB, ra, rb, {j.impulseX, j.impulseY, j.impulseZ});
            ApplyAngularImpulse(a, b, invIA, invIB, {j.angularImpulseX, j.angularImpulseY, j.angularImpulseZ});
        }

        for (PrismaticJoint& j : prismaticJoints_) {
            const float sum = std::abs(j.impulseT1) + std::abs(j.impulseT2) + std::abs(j.impulseAxis) + std::abs(j.motorImpulseSum);
            if (sum <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Mat3 invIA = a.InvInertiaWorld();
            const Mat3 invIB = b.InvInertiaWorld();
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            Vec3 axis = Normalize(Rotate(a.orientation, j.localAxisA));
            Vec3 t1 = Cross(axis, {0.0f, 1.0f, 0.0f});
            if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axis, {0.0f, 0.0f, 1.0f});
            t1 = Normalize(t1);
            const Vec3 t2 = Normalize(Cross(axis, t1));
            const Vec3 impulse = j.impulseT1 * t1 + j.impulseT2 * t2 + (j.impulseAxis + j.motorImpulseSum) * axis;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
        }

        for (ServoJoint& j : servoJoints_) {
            const float linearSum = std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ);
            const float angularSum = std::abs(j.angularImpulse1) + std::abs(j.angularImpulse2) + std::abs(j.servoImpulseSum);
            if (linearSum + angularSum <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Mat3 invIA = a.InvInertiaWorld();
            const Mat3 invIB = b.InvInertiaWorld();
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            ApplyImpulse(a, b, invIA, invIB, ra, rb, {j.impulseX, j.impulseY, j.impulseZ});

            const Vec3 axisA = Normalize(Rotate(a.orientation, j.localAxisA));
            Vec3 t1 = Cross(axisA, {1.0f, 0.0f, 0.0f});
            if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axisA, {0.0f, 0.0f, 1.0f});
            t1 = Normalize(t1);
            const Vec3 t2 = Normalize(Cross(axisA, t1));
            ApplyAngularImpulse(a, b, invIA, invIB, j.angularImpulse1 * t1);
            ApplyAngularImpulse(a, b, invIA, invIB, j.angularImpulse2 * t2);
            ApplyAngularImpulse(a, b, invIA, invIB, j.servoImpulseSum * axisA);
        }
    }

void World::SolveNormalScalar(Contact& c) {

        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];

        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = c.point - a.position;
        const Vec3 rb = c.point - b.position;

        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const Vec3 relativeVelocity = vb - va;
        const float separatingVelocity = Dot(relativeVelocity, c.normal);

        const float speedIntoContact = -separatingVelocity;
        const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);
        const Vec3 raCrossN = Cross(ra, c.normal);
        const Vec3 rbCrossN = Cross(rb, c.normal);
        const float angularTermA = Dot(raCrossN, invIA * raCrossN);
        const float angularTermB = Dot(rbCrossN, invIB * rbCrossN);
        const float normalMass = a.invMass + b.invMass + angularTermA + angularTermB;
        if (normalMass <= kEpsilon) {
            return;
        }

        float biasTerm = 0.0f;
        float penetration = c.penetration;
        if (c.anchorsValid && c.persistenceAge >= contactSolverConfig_.manifoldAnchorReuseMinAge) {
            float anchorPenetration = 0.0f;
            if (TryComputeAnchorSeparation(c, anchorPenetration)) {
                penetration = anchorPenetration;
#ifndef NDEBUG
                ++solverTelemetry_.anchorReuseHitCount;
#endif
            } else {
#ifndef NDEBUG
                ++solverTelemetry_.anchorReuseFallbackCount;
#endif
            }
        }
        const float penetrationError = std::max(penetration - contactSolverConfig_.penetrationSlop, 0.0f);
        const float massRatioBoost = ComputeHighMassRatioBoost(a, b);
        if (contactSolverConfig_.useSplitImpulse && !solverRelaxationPassActive_) {
            if (penetrationError > 0.0f) {
                if (normalMass > kEpsilon) {
                    const float boostedFactor = contactSolverConfig_.splitImpulseCorrectionFactor
                        * (1.0f + contactSolverConfig_.highMassRatioSplitImpulseBoost * (massRatioBoost - 1.0f));
                    const float correctionMagnitude = boostedFactor * penetrationError / normalMass;
                    const Vec3 correction = correctionMagnitude * c.normal;
                    AccumulateSplitImpulseCorrection(c.a, -correction * a.invMass, {0.0f, 0.0f, 0.0f});
                    AccumulateSplitImpulseCorrection(c.b, correction * b.invMass, {0.0f, 0.0f, 0.0f});
                }
            }
        } else if (currentSubstepDt_ > kEpsilon && !solverRelaxationPassActive_) {
            const float maxSafeSeparatingSpeed = penetrationError / currentSubstepDt_;
            if (separatingVelocity <= maxSafeSeparatingSpeed) {
                const float boostedBias = contactSolverConfig_.penetrationBiasFactor
                    * (1.0f + contactSolverConfig_.highMassRatioBiasBoost * (massRatioBoost - 1.0f));
                biasTerm = (boostedBias * penetrationError) / currentSubstepDt_;
                biasTerm = std::min(biasTerm, std::max(contactSolverConfig_.penetrationBiasMaxSpeed, 0.0f));
            }
        }
        if (contactSolverConfig_.softContactBiasRate > 0.0f
            && contactSolverConfig_.softContactCompliance > 0.0f
            && c.persistenceAge >= contactSolverConfig_.softContactMinAge
            && std::abs(separatingVelocity) <= contactSolverConfig_.softContactMaxNormalSpeed
            && separatingVelocity <= 0.0f) {
            const float softBias = (contactSolverConfig_.softContactBiasRate * penetrationError)
                / std::max(currentSubstepDt_, kEpsilon);
            const float softenedMass = normalMass + contactSolverConfig_.softContactCompliance;
            const float lambdaSoft = (softBias - separatingVelocity) / std::max(softenedMass, kEpsilon);
            const float oldNormalImpulse = c.normalImpulseSum;
            c.normalImpulseSum = std::max(0.0f, c.normalImpulseSum + lambdaSoft);
            const float softDelta = c.normalImpulseSum - oldNormalImpulse;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, softDelta * c.normal);
            return;
        }

        float lambdaN = -(1.0f + restitution) * separatingVelocity / normalMass;
        if (biasTerm > 0.0f) {
            lambdaN += biasTerm / normalMass;
        }
        const float oldNormalImpulse = c.normalImpulseSum;
        c.normalImpulseSum = std::max(0.0f, c.normalImpulseSum + lambdaN);
        lambdaN = c.normalImpulseSum - oldNormalImpulse;
        ApplyImpulse(a, b, invIA, invIB, ra, rb, lambdaN * c.normal);
    }

void World::ResetBlockSolveDebugStep(Manifold& manifold) {

        manifold.blockSolveDebug.selectedPreNormalImpulses = {0.0f, 0.0f};
        manifold.blockSolveDebug.selectedPostNormalImpulses = {0.0f, 0.0f};
        manifold.blockSolveDebug.selectedPairPenetrationStep = 0.0f;
    }

void World::IncrementBlockSolveFallbackCounter(Manifold& manifold, BlockSolveFallbackReason reason) {

        switch (reason) {
            case BlockSolveFallbackReason::Ineligible:
                ++manifold.blockSolveDebug.scalarFallbackIneligibleCount;
                break;
            case BlockSolveFallbackReason::TypePolicy:
            case BlockSolveFallbackReason::QualityGate:
            case BlockSolveFallbackReason::PersistenceGate:
                ++manifold.blockSolveDebug.scalarFallbackPersistenceGateCount;
                break;
            case BlockSolveFallbackReason::InvalidManifoldNormal:
                ++manifold.blockSolveDebug.scalarFallbackInvalidNormalCount;
                break;
            case BlockSolveFallbackReason::ContactNormalMismatch:
                ++manifold.blockSolveDebug.scalarFallbackNormalMismatchCount;
                break;
            case BlockSolveFallbackReason::MissingBlockSlots:
                ++manifold.blockSolveDebug.scalarFallbackMissingSlotsCount;
                break;
            case BlockSolveFallbackReason::DegenerateMassMatrix:
                ++manifold.blockSolveDebug.scalarFallbackDegenerateMassCount;
                break;
            case BlockSolveFallbackReason::ConditionEstimateExceeded:
                ++manifold.blockSolveDebug.scalarFallbackConditionEstimateCount;
                break;
            case BlockSolveFallbackReason::LcpFailure:
                ++manifold.blockSolveDebug.scalarFallbackLcpFailureCount;
                break;
            case BlockSolveFallbackReason::NonFiniteResult:
                ++manifold.blockSolveDebug.scalarFallbackNonFiniteCount;
                break;
            case BlockSolveFallbackReason::None:
                break;
        }
    }

void World::IncrementFallbackReasonTelemetry(SolverTelemetry::FallbackReasonCounters& counters, BlockSolveFallbackReason reason) {

        switch (reason) {
            case BlockSolveFallbackReason::Ineligible:
                ++counters.ineligible;
                break;
            case BlockSolveFallbackReason::TypePolicy:
            case BlockSolveFallbackReason::QualityGate:
            case BlockSolveFallbackReason::PersistenceGate:
                ++counters.persistenceGate;
                break;
            case BlockSolveFallbackReason::InvalidManifoldNormal:
                ++counters.invalidManifoldNormal;
                break;
            case BlockSolveFallbackReason::ContactNormalMismatch:
                ++counters.contactNormalMismatch;
                break;
            case BlockSolveFallbackReason::MissingBlockSlots:
                ++counters.missingBlockSlots;
                break;
            case BlockSolveFallbackReason::DegenerateMassMatrix:
                ++counters.degenerateMassMatrix;
                break;
            case BlockSolveFallbackReason::ConditionEstimateExceeded:
                ++counters.conditionEstimateExceeded;
                break;
            case BlockSolveFallbackReason::LcpFailure:
                ++counters.lcpFailure;
                break;
            case BlockSolveFallbackReason::NonFiniteResult:
                ++counters.nonFiniteResult;
                break;
            case BlockSolveFallbackReason::None:
                ++counters.none;
                break;
        }
    }

void World::RecordManifoldSolveTelemetry(const Manifold& manifold,
                                      BlockSolveFallbackReason fallbackReason,
                                      float determinantOrConditionEstimate,
                                      float impulseContinuityMetric) {

        auto accumulate = [&](SolverTelemetry::ManifoldSolveBucket& bucket) {
            ++bucket.solveCount;
            bucket.manifoldContactCount += manifold.contacts.size();
            int selectedBlockSize = 0;
            for (int idx : manifold.selectedBlockContactIndices) {
                if (idx >= 0 && static_cast<std::size_t>(idx) < manifold.contacts.size()) {
                    ++selectedBlockSize;
                }
            }
            bucket.selectedBlockSize += static_cast<std::uint64_t>(selectedBlockSize);
            bucket.blockUsed += manifold.usedBlockSolve ? 1u : 0u;
            bucket.fallbackUsed += (!manifold.usedBlockSolve) ? 1u : 0u;
            IncrementFallbackReasonTelemetry(bucket.fallbackReason, fallbackReason);
            if (std::isfinite(determinantOrConditionEstimate)) {
                bucket.determinantOrConditionEstimate += static_cast<double>(determinantOrConditionEstimate);
                ++bucket.determinantOrConditionEstimateSamples;
            }
            if (std::isfinite(impulseContinuityMetric)) {
                bucket.impulseContinuityMetric += static_cast<double>(impulseContinuityMetric);
                ++bucket.impulseContinuityMetricSamples;
            }
        };

        accumulate(solverTelemetry_.manifoldSolveScope);
        accumulate(solverTelemetry_.manifoldTypeBuckets[manifold.manifoldType]);
    }

bool World::IsValidBlockContactPoint(const Contact& contact) {

        if (!std::isfinite(contact.penetration) || contact.penetration <= 0.0f) {
            return false;
        }
        if (!std::isfinite(contact.normal.x) || !std::isfinite(contact.normal.y) || !std::isfinite(contact.normal.z)) {
            return false;
        }
        return LengthSquared(contact.normal) > kEpsilon;
    }

bool World::ContactPairStableOrder(const Contact& lhs, const Contact& rhs) {

        if (lhs.key != rhs.key) {
            return lhs.key < rhs.key;
        }
        return StableContactFallbackKey(lhs) <= StableContactFallbackKey(rhs);
    }

std::array<std::uint64_t, 2> World::SortedContactKeyPair(std::uint64_t k0, std::uint64_t k1) {

        if (k1 < k0) {
            std::swap(k0, k1);
        }
        return {k0, k1};
    }

void World::RecordSelectedPairHistory(const Manifold& manifold) {

        if (manifold.selectedBlockContactIndices[0] < 0 || manifold.selectedBlockContactIndices[1] < 0) {
            return;
        }
        const ManifoldKey manifoldId = MakeManifoldId(manifold.a, manifold.b, manifold.manifoldType);
        const std::array<std::uint64_t, 2> currentPair =
            SortedContactKeyPair(manifold.selectedBlockContactKeys[0], manifold.selectedBlockContactKeys[1]);
        SelectionHistory& history = selectedPairHistory_[manifoldId];
        if (history.hasPrevious && history.hasLast && currentPair == history.previousPair && currentPair != history.lastPair) {
            ++solverTelemetry_.selectedPairOscillationEvents;
            if (debugContactPersistence_) {
                std::fprintf(DebugLogStream(),
                             "[minphys3d] frame=%llu selected_pair_oscillation pair=(%u,%u) type=%u prev=(%llu,%llu) last=(%llu,%llu) current=(%llu,%llu)\n",
                             static_cast<unsigned long long>(debugFrameIndex_),
                             manifold.a,
                             manifold.b,
                             static_cast<unsigned>(manifold.manifoldType),
                             static_cast<unsigned long long>(history.previousPair[0]),
                             static_cast<unsigned long long>(history.previousPair[1]),
                             static_cast<unsigned long long>(history.lastPair[0]),
                             static_cast<unsigned long long>(history.lastPair[1]),
                             static_cast<unsigned long long>(currentPair[0]),
                             static_cast<unsigned long long>(currentPair[1]));
            }
        }
        history.previousPair = history.lastPair;
        history.hasPrevious = history.hasLast;
        history.lastPair = currentPair;
        history.hasLast = true;
    }

void World::DebugLogManifoldContactKeys(const Manifold& manifold) const {

        if (!debugContactPersistence_) {
            return;
        }
        std::fprintf(DebugLogStream(),
                     "[minphys3d] frame=%llu manifold pair=(%u,%u) type=%u contact_keys=",
                     static_cast<unsigned long long>(debugFrameIndex_),
                     manifold.a,
                     manifold.b,
                     static_cast<unsigned>(manifold.manifoldType));
        if (manifold.contacts.empty()) {
            std::fprintf(DebugLogStream(), "none");
        } else {
            for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
                const Contact& c = manifold.contacts[i];
                std::fprintf(DebugLogStream(),
                             "%s%llu(age=%u)",
                             (i == 0 ? "" : ","),
                             static_cast<unsigned long long>(c.key),
                             static_cast<unsigned>(c.persistenceAge));
            }
        }
        std::fprintf(DebugLogStream(), "\n");
    }

void World::DebugLogContactTransitions(const Manifold& previous, const Manifold& current) const {

        if (!debugContactPersistence_) {
            return;
        }
        std::unordered_map<std::uint64_t, int> previousCounts;
        std::unordered_map<std::uint64_t, int> currentCounts;
        for (const Contact& c : previous.contacts) {
            ++previousCounts[c.key];
        }
        for (const Contact& c : current.contacts) {
            ++currentCounts[c.key];
        }
        for (const auto& [key, oldCount] : previousCounts) {
            const int newCount = currentCounts.count(key) > 0 ? currentCounts.at(key) : 0;
            if (newCount < oldCount) {
                std::fprintf(DebugLogStream(),
                             "[minphys3d] frame=%llu manifold pair=(%u,%u) type=%u contact_remove key=%llu count=%d\n",
                             static_cast<unsigned long long>(debugFrameIndex_),
                             current.a,
                             current.b,
                             static_cast<unsigned>(current.manifoldType),
                             static_cast<unsigned long long>(key),
                             oldCount - newCount);
            }
        }
        for (const auto& [key, newCount] : currentCounts) {
            const int oldCount = previousCounts.count(key) > 0 ? previousCounts.at(key) : 0;
            if (newCount > oldCount) {
                std::fprintf(DebugLogStream(),
                             "[minphys3d] frame=%llu manifold pair=(%u,%u) type=%u contact_add key=%llu count=%d\n",
                             static_cast<unsigned long long>(debugFrameIndex_),
                             current.a,
                             current.b,
                             static_cast<unsigned>(current.manifoldType),
                             static_cast<unsigned long long>(key),
                             newCount - oldCount);
            }
        }
    }

void World::SelectBlockSolvePair(Manifold& manifold) const {

        // Geometry-first pair selection tuning knobs for 3+ contact manifolds.
        constexpr float kPairScoreEpsilon = 1e-6f;
        constexpr float kSupportAreaProxyScale = 0.25f;
        constexpr float kQualityMinSpreadSq = 1e-6f;
        constexpr float kQualityMinSupportAreaProxy = 1e-7f;
        constexpr float kQualityMinLeverSq = 1e-8f;
        constexpr float kQualityMinPenetrationSum = 1e-6f;

        const std::array<std::uint64_t, 2> previousSelectedKeys = manifold.selectedBlockContactKeys;
        manifold.selectedBlockContactIndices = {-1, -1};
        manifold.selectedBlockContactKeys = {0u, 0u};
        manifold.selectedBlockPairPersistent = false;
        manifold.selectedBlockPairQualityPass = false;
#ifndef NDEBUG
        const auto logSelection = [&]() {
            if (!debugContactPersistence_) {
                return;
            }
            std::fprintf(DebugLogStream(),
                         "[minphys3d] frame=%llu selected_block pair=(%u,%u) type=%u idx=(%d,%d) keys=(%llu,%llu)\n",
                         static_cast<unsigned long long>(debugFrameIndex_),
                         manifold.a,
                         manifold.b,
                         static_cast<unsigned>(manifold.manifoldType),
                         manifold.selectedBlockContactIndices[0],
                         manifold.selectedBlockContactIndices[1],
                         static_cast<unsigned long long>(manifold.selectedBlockContactKeys[0]),
                         static_cast<unsigned long long>(manifold.selectedBlockContactKeys[1]));
        };
#endif

        if (manifold.contacts.size() < 2) {
#ifndef NDEBUG
            logSelection();
#endif
            return;
        }

        if (manifold.contacts.size() == 2) {
            manifold.selectedBlockContactIndices = {0, 1};
            manifold.selectedBlockContactKeys = {manifold.contacts[0].key, manifold.contacts[1].key};
            manifold.selectedBlockPairPersistent = true;
            manifold.selectedBlockPairQualityPass = true;
#ifndef NDEBUG
            logSelection();
#endif
            return;
        }

        struct PairSelection {
            int i = -1;
            int j = -1;
            float supportAreaProxy = -1.0f;
            float penetration = -1.0f;
            float spread = -1.0f;
            float lever = -1.0f;
            int ageMin = -1;
            int ageSum = -1;
            float continuity = 0.0f;
            std::array<std::uint64_t, 2> sortedKeys{0u, 0u};
        };

        const auto computePairQuality = [&](const Contact& a, const Contact& b) {
            const float penetration = a.penetration + b.penetration;
            const float spread = LengthSquared(a.point - b.point);
            const float supportAreaProxy = spread * std::max(0.0f, penetration) * kSupportAreaProxyScale;
            const Vec3 midPoint = 0.5f * (a.point + b.point);
            const Vec3 centers = 0.5f * (bodies_[manifold.a].position + bodies_[manifold.b].position);
            const float lever = LengthSquared(midPoint - centers);
            return spread >= kQualityMinSpreadSq
                && supportAreaProxy >= kQualityMinSupportAreaProxy
                && lever >= kQualityMinLeverSq
                && penetration >= kQualityMinPenetrationSum;
        };

        if (previousSelectedKeys[0] != 0u && previousSelectedKeys[1] != 0u) {
            int prevIdx0 = -1;
            int prevIdx1 = -1;
            for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
                const std::uint64_t key = manifold.contacts[i].key;
                if (key == previousSelectedKeys[0] && prevIdx0 < 0) {
                    prevIdx0 = static_cast<int>(i);
                } else if (key == previousSelectedKeys[1] && prevIdx1 < 0) {
                    prevIdx1 = static_cast<int>(i);
                }
            }
            if (prevIdx0 >= 0 && prevIdx1 >= 0 && prevIdx0 != prevIdx1) {
                const Contact& c0 = manifold.contacts[static_cast<std::size_t>(prevIdx0)];
                const Contact& c1 = manifold.contacts[static_cast<std::size_t>(prevIdx1)];
                if (IsValidBlockContactPoint(c0)
                    && IsValidBlockContactPoint(c1)
                    && computePairQuality(c0, c1)) {
                    int idx0 = prevIdx0;
                    int idx1 = prevIdx1;
                    if (!ContactPairStableOrder(c0, c1)) {
                        std::swap(idx0, idx1);
                    }
                    manifold.selectedBlockContactIndices = {idx0, idx1};
                    manifold.selectedBlockContactKeys = {
                        manifold.contacts[static_cast<std::size_t>(idx0)].key,
                        manifold.contacts[static_cast<std::size_t>(idx1)].key};
                    manifold.selectedBlockPairPersistent =
                        manifold.contacts[static_cast<std::size_t>(idx0)].persistenceAge > 0
                        && manifold.contacts[static_cast<std::size_t>(idx1)].persistenceAge > 0;
                    manifold.selectedBlockPairQualityPass = true;
#ifndef NDEBUG
                    logSelection();
#endif
                    return;
                }
            }
        }

        const Vec3 centers = 0.5f * (bodies_[manifold.a].position + bodies_[manifold.b].position);
        PairSelection best{};
        for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
            const Contact& a = manifold.contacts[static_cast<std::size_t>(i)];
            if (!IsValidBlockContactPoint(a)) {
                continue;
            }
            for (std::size_t j = i + 1; j < manifold.contacts.size(); ++j) {
                const Contact& b = manifold.contacts[static_cast<std::size_t>(j)];
                if (!IsValidBlockContactPoint(b)) {
                    continue;
                }

                PairSelection candidate;
                candidate.i = static_cast<int>(i);
                candidate.j = static_cast<int>(j);
                candidate.penetration = a.penetration + b.penetration;
                candidate.spread = LengthSquared(a.point - b.point);
                candidate.supportAreaProxy =
                    candidate.spread * std::max(0.0f, candidate.penetration) * kSupportAreaProxyScale;
                const Vec3 midPoint = 0.5f * (a.point + b.point);
                candidate.lever = LengthSquared(midPoint - centers);
                candidate.ageMin = static_cast<int>(std::min(a.persistenceAge, b.persistenceAge));
                candidate.ageSum = static_cast<int>(a.persistenceAge + b.persistenceAge);
                const bool keyMatch = previousSelectedKeys[0] != 0u
                    && previousSelectedKeys[1] != 0u
                    && (a.key == previousSelectedKeys[0] || a.key == previousSelectedKeys[1])
                    && (b.key == previousSelectedKeys[0] || b.key == previousSelectedKeys[1]);
                candidate.continuity = keyMatch ? 1.0f : 0.0f;
                candidate.sortedKeys = SortedContactKeyPair(a.key, b.key);

                const bool betterSupportAreaProxy =
                    candidate.supportAreaProxy > (best.supportAreaProxy + kPairScoreEpsilon);
                const bool tieSupportAreaProxy =
                    std::abs(candidate.supportAreaProxy - best.supportAreaProxy) <= kPairScoreEpsilon;
                const bool betterAgeMin = tieSupportAreaProxy && candidate.ageMin > best.ageMin;
                const bool tieAgeMin = tieSupportAreaProxy && candidate.ageMin == best.ageMin;
                const bool betterAgeSum = tieAgeMin && candidate.ageSum > best.ageSum;
                const bool tieAgeSum = tieAgeMin && candidate.ageSum == best.ageSum;
                const bool betterContinuity =
                    tieAgeSum && candidate.continuity > (best.continuity + kPairScoreEpsilon);
                const bool tieContinuity =
                    tieAgeSum && std::abs(candidate.continuity - best.continuity) <= kPairScoreEpsilon;
                const bool betterLever = tieContinuity && candidate.lever > (best.lever + kPairScoreEpsilon);
                const bool tieLever = tieContinuity && std::abs(candidate.lever - best.lever) <= kPairScoreEpsilon;
                const bool betterPenetration = tieLever && candidate.penetration > (best.penetration + kPairScoreEpsilon);
                const bool tiePenetration =
                    tieLever && std::abs(candidate.penetration - best.penetration) <= kPairScoreEpsilon;
                const bool betterDeterministic =
                    tiePenetration && (best.i < 0 || candidate.sortedKeys < best.sortedKeys);

                if (best.i < 0
                    || betterSupportAreaProxy
                    || betterAgeMin
                    || betterAgeSum
                    || betterContinuity
                    || betterLever
                    || betterPenetration
                    || betterDeterministic) {
                    best = candidate;
                }
            }
        }

        if (best.i < 0 || best.j < 0) {
#ifndef NDEBUG
            logSelection();
#endif
            return;
        }

        int idx0 = best.i;
        int idx1 = best.j;
        const Contact& c0 = manifold.contacts[static_cast<std::size_t>(idx0)];
        const Contact& c1 = manifold.contacts[static_cast<std::size_t>(idx1)];
        if (!ContactPairStableOrder(c0, c1)) {
            std::swap(idx0, idx1);
        }

        manifold.selectedBlockContactIndices = {idx0, idx1};
        manifold.selectedBlockContactKeys = {
            manifold.contacts[static_cast<std::size_t>(idx0)].key,
            manifold.contacts[static_cast<std::size_t>(idx1)].key};
        manifold.selectedBlockPairPersistent =
            manifold.contacts[static_cast<std::size_t>(idx0)].persistenceAge > 0
            && manifold.contacts[static_cast<std::size_t>(idx1)].persistenceAge > 0;
        manifold.selectedBlockPairQualityPass =
            best.spread >= kQualityMinSpreadSq
            && best.supportAreaProxy >= kQualityMinSupportAreaProxy
            && best.lever >= kQualityMinLeverSq
            && best.penetration >= kQualityMinPenetrationSum;

#ifndef NDEBUG
        logSelection();
#endif
    }

bool World::IsBlockSolveEligible(const Manifold& manifold, BlockSolveFallbackReason* outIneligibleReason) const {

        if (outIneligibleReason != nullptr) {
            *outIneligibleReason = BlockSolveFallbackReason::Ineligible;
        }
        if (manifold.contacts.size() < 2) {
            return false;
        }

        const std::uint32_t typeBit = (manifold.manifoldType < 32u) ? (1u << manifold.manifoldType) : 0u;
        if (typeBit == 0u || (contactSolverConfig_.blockManifoldTypeMask & typeBit) == 0u) {
            if (outIneligibleReason != nullptr) {
                *outIneligibleReason = BlockSolveFallbackReason::TypePolicy;
            }
            return false;
        }

        if (manifold.lowQuality) {
            if (outIneligibleReason != nullptr) {
                *outIneligibleReason = BlockSolveFallbackReason::QualityGate;
            }
            return false;
        }
        const std::uint8_t configuredMinAge =
            contactSolverConfig_.blockMinPersistenceByType[static_cast<std::size_t>(manifold.manifoldType)];
        const std::uint8_t requiredAge = static_cast<std::uint8_t>(
            std::max<std::uint8_t>(configuredMinAge, manifold.contacts.size() >= 3 ? 1u : 0u));
        if (requiredAge > 0) {
            const int idx0 = manifold.selectedBlockContactIndices[0];
            const int idx1 = manifold.selectedBlockContactIndices[1];
            if (idx0 < 0 || idx1 < 0 || idx0 == idx1
                || static_cast<std::size_t>(std::max(idx0, idx1)) >= manifold.contacts.size()) {
                if (outIneligibleReason != nullptr) {
                    *outIneligibleReason = BlockSolveFallbackReason::PersistenceGate;
                }
                return false;
            }
            const Contact& p0 = manifold.contacts[static_cast<std::size_t>(idx0)];
            const Contact& p1 = manifold.contacts[static_cast<std::size_t>(idx1)];
            if (p0.persistenceAge < requiredAge || p1.persistenceAge < requiredAge) {
                if (outIneligibleReason != nullptr) {
                    *outIneligibleReason = BlockSolveFallbackReason::PersistenceGate;
                }
                return false;
            }
        } else if (manifold.contacts.size() >= 3 && !manifold.selectedBlockPairPersistent) {
            if (outIneligibleReason != nullptr) {
                *outIneligibleReason = BlockSolveFallbackReason::PersistenceGate;
            }
            return false;
        }
        if (!manifold.selectedBlockPairQualityPass) {
            if (outIneligibleReason != nullptr) {
                *outIneligibleReason = BlockSolveFallbackReason::QualityGate;
            }
            return false;
        }
        const Vec3 manifoldNormal = Normalize(manifold.normal);
        if (!std::isfinite(manifoldNormal.x) || !std::isfinite(manifoldNormal.y) || !std::isfinite(manifoldNormal.z)) {
            return false;
        }
        if (LengthSquared(manifoldNormal) <= kEpsilon) {
            return false;
        }

        const int idx0 = manifold.selectedBlockContactIndices[0];
        const int idx1 = manifold.selectedBlockContactIndices[1];
        if (idx0 < 0 || idx1 < 0 || idx0 == idx1
            || static_cast<std::size_t>(std::max(idx0, idx1)) >= manifold.contacts.size()) {
            return false;
        }

        const Contact& selected0 = manifold.contacts[static_cast<std::size_t>(idx0)];
        const Contact& selected1 = manifold.contacts[static_cast<std::size_t>(idx1)];
        for (const Contact* contact : {&selected0, &selected1}) {
            if (!IsValidBlockContactPoint(*contact)) {
                return false;
            }
            if (Dot(contact->normal, manifoldNormal) < 0.95f) {
                return false;
            }
        }

        const int slot0 = FindBlockSlot(manifold, selected0.key);
        const int slot1 = FindBlockSlot(manifold, selected1.key);
        if (slot0 < 0 || slot1 < 0 || slot0 == slot1) {
            return false;
        }
        return true;
    }

bool World::EnsureStableTwoPointOrder(Manifold& manifold) {

        if (manifold.contacts.size() != 2) {
            return false;
        }
        const Contact& c0 = manifold.contacts[0];
        const Contact& c1 = manifold.contacts[1];
        const bool orderedByFeature = c0.key < c1.key;
        const bool sameFeatureUsesFallback = c0.key == c1.key
            && StableContactFallbackKey(c0) <= StableContactFallbackKey(c1);
        if (orderedByFeature || sameFeatureUsesFallback) {
            return false;
        }
        std::swap(manifold.contacts[0], manifold.contacts[1]);
#ifndef NDEBUG
        ++solverTelemetry_.reorderDetected;
#endif
        return true;
    }

bool World::IsFace4PointBlockEligible(const Manifold& manifold, BlockSolveFallbackReason* outReason) {

        if (outReason != nullptr) {
            *outReason = BlockSolveFallbackReason::Ineligible;
        }
        if (!contactSolverConfig_.useFace4PointNormalBlock) {
            return false;
        }
        if (manifold.manifoldType != 9 || manifold.contacts.size() != 4) {
            if (outReason != nullptr) {
                *outReason = BlockSolveFallbackReason::TypePolicy;
            }
            return false;
        }
        if (manifold.lowQuality || !manifold.selectedBlockPairQualityPass) {
            if (outReason != nullptr) {
                *outReason = BlockSolveFallbackReason::QualityGate;
            }
            return false;
        }
        const int idx0 = manifold.selectedBlockContactIndices[0];
        const int idx1 = manifold.selectedBlockContactIndices[1];
        if (idx0 < 0 || idx1 < 0 || idx0 == idx1
            || static_cast<std::size_t>(std::max(idx0, idx1)) >= manifold.contacts.size()) {
            if (outReason != nullptr) {
                *outReason = BlockSolveFallbackReason::PersistenceGate;
            }
            return false;
        }
        const Contact& selected0 = manifold.contacts[static_cast<std::size_t>(idx0)];
        const Contact& selected1 = manifold.contacts[static_cast<std::size_t>(idx1)];
        if (selected0.persistenceAge < contactSolverConfig_.face4MinPersistenceAge
            || selected1.persistenceAge < contactSolverConfig_.face4MinPersistenceAge) {
            if (outReason != nullptr) {
                *outReason = BlockSolveFallbackReason::PersistenceGate;
            }
            return false;
        }
        for (const Contact& c : manifold.contacts) {
            if (c.persistenceAge < contactSolverConfig_.face4MinPersistenceAge || c.key == 0u) {
                if (outReason != nullptr) {
                    *outReason = BlockSolveFallbackReason::PersistenceGate;
                }
                return false;
            }
        }
        if (contactSolverConfig_.face4RequireFrictionCoherence) {
            if (!manifold.tangentBasisValid
                || !manifold.manifoldTangentImpulseValid
                || !manifold.selectedBlockPairPersistent
                || !manifold.selectedBlockPairQualityPass) {
                if (outReason != nullptr) {
                    *outReason = BlockSolveFallbackReason::PersistenceGate;
                }
#ifndef NDEBUG
                ++solverTelemetry_.face4BlockedByFrictionCoherenceGate;
#endif
                return false;
            }
            int stableContacts = 0;
            for (const Contact& c : manifold.contacts) {
                if (c.persistenceAge >= contactSolverConfig_.face4MinCoherentPersistenceAge) {
                    ++stableContacts;
                }
            }
            if (stableContacts < static_cast<int>(contactSolverConfig_.face4MinStableContactCount)) {
                if (outReason != nullptr) {
                    *outReason = BlockSolveFallbackReason::PersistenceGate;
                }
#ifndef NDEBUG
                ++solverTelemetry_.face4BlockedByFrictionCoherenceGate;
#endif
                return false;
            }
        }
        return true;
    }

void World::SolveManifoldNormalImpulses(Manifold& manifold) {

        if (manifold.contacts.empty()) {
            return;
        }

#ifndef NDEBUG
        ResetBlockSolveDebugStep(manifold);
        const int selectedIdx0 = manifold.selectedBlockContactIndices[0];
        const int selectedIdx1 = manifold.selectedBlockContactIndices[1];
        if (selectedIdx0 >= 0 && selectedIdx1 >= 0
            && selectedIdx0 != selectedIdx1
            && static_cast<std::size_t>(std::max(selectedIdx0, selectedIdx1)) < manifold.contacts.size()) {
            const Contact& selected0 = manifold.contacts[static_cast<std::size_t>(selectedIdx0)];
            const Contact& selected1 = manifold.contacts[static_cast<std::size_t>(selectedIdx1)];
            manifold.blockSolveDebug.selectedPreNormalImpulses = {selected0.normalImpulseSum, selected1.normalImpulseSum};
            manifold.blockSolveDebug.selectedPairPenetrationStep = selected0.penetration + selected1.penetration;
        }
#endif

        BlockSolveFallbackReason ineligibleReason = BlockSolveFallbackReason::Ineligible;
        manifold.blockSolveEligible = contactSolverConfig_.useBlockSolver
            && IsBlockSolveEligible(manifold, &ineligibleReason);
        manifold.usedBlockSolve = false;

#ifndef NDEBUG
        if (manifold.blockSolveEligible) {
            ++solverTelemetry_.blockSolveEligible;
        }
#endif

        if (!manifold.blockSolveEligible) {
#ifndef NDEBUG
            ++solverTelemetry_.scalarPathIneligible;
            if (ineligibleReason == BlockSolveFallbackReason::TypePolicy) {
                ++solverTelemetry_.blockRejectedByTypePolicy;
            } else if (ineligibleReason == BlockSolveFallbackReason::QualityGate
                       || ineligibleReason == BlockSolveFallbackReason::PersistenceGate) {
                ++solverTelemetry_.blockRejectedByQualityOrPersistence;
            }
            if (ineligibleReason == BlockSolveFallbackReason::PersistenceGate) {
                ++solverTelemetry_.scalarFallbackPersistenceGate;
            }
            IncrementBlockSolveFallbackCounter(manifold, ineligibleReason);
            if (debugBlockSolveRouting_) {
                std::fprintf(
                    DebugLogStream(),
                    "[minphys3d] scalar route (ineligible) pair=(%u,%u) type=%u contacts=%zu lowQuality=%d reason=%d persistent=%d\n",
                    manifold.a,
                    manifold.b,
                    static_cast<unsigned>(manifold.manifoldType),
                    manifold.contacts.size(),
                    manifold.lowQuality ? 1 : 0,
                    static_cast<int>(ineligibleReason),
                    manifold.selectedBlockPairPersistent ? 1 : 0);
            }
#endif
            for (Contact& c : manifold.contacts) {
                SolveNormalScalar(c);
            }
#ifndef NDEBUG
            if (selectedIdx0 >= 0 && selectedIdx1 >= 0
                && static_cast<std::size_t>(std::max(selectedIdx0, selectedIdx1)) < manifold.contacts.size()) {
                manifold.blockSolveDebug.selectedPostNormalImpulses = {
                    manifold.contacts[static_cast<std::size_t>(selectedIdx0)].normalImpulseSum,
                    manifold.contacts[static_cast<std::size_t>(selectedIdx1)].normalImpulseSum};
            }
            const float impulseContinuityMetric = std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[0] - manifold.blockSolveDebug.selectedPreNormalImpulses[0])
                + std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[1] - manifold.blockSolveDebug.selectedPreNormalImpulses[1]);
            RecordManifoldSolveTelemetry(manifold, ineligibleReason, std::numeric_limits<float>::quiet_NaN(), impulseContinuityMetric);
#endif
            return;
        }

        BlockSolveFallbackReason fallbackReason = BlockSolveFallbackReason::None;
        float determinantOrConditionEstimate = std::numeric_limits<float>::quiet_NaN();
        bool blockSolved = false;
        bool usedFace4 = false;
        if (contactSolverConfig_.useFace4PointNormalBlock
            && IsFace4PointBlockEligible(manifold, &fallbackReason)) {
#ifndef NDEBUG
            ++solverTelemetry_.face4Attempted;
#endif
            blockSolved = SolveNormalProjected4(manifold, fallbackReason, determinantOrConditionEstimate);
            if (blockSolved) {
                usedFace4 = true;
#ifndef NDEBUG
                ++solverTelemetry_.face4Used;
#endif
            } else {
#ifndef NDEBUG
                ++solverTelemetry_.face4FallbackToBlock2;
#endif
            }
        }
        if (!blockSolved) {
            blockSolved = SolveNormalBlock2(manifold, fallbackReason, determinantOrConditionEstimate);
        }
        if (blockSolved) {
            manifold.usedBlockSolve = true;
#ifndef NDEBUG
            ++solverTelemetry_.blockSolveUsed;
            ++manifold.blockSolveDebug.blockSolveUsedCount;
            if (debugBlockSolveRouting_) {
                std::fprintf(
                    DebugLogStream(),
                    "[minphys3d] block route pair=(%u,%u) type=%u contacts=%zu\n",
                    manifold.a,
                    manifold.b,
                    static_cast<unsigned>(manifold.manifoldType),
                    manifold.contacts.size());
            }
#endif
            // Face-4 projected solve already resolves all contact normals in a 4-point manifold.
            // Only run scalar extras for Block2, where the selected pair covers a strict subset.
            if (!usedFace4) {
                const int blockIdx0 = manifold.selectedBlockContactIndices[0];
                const int blockIdx1 = manifold.selectedBlockContactIndices[1];
                for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
                    if (static_cast<int>(i) == blockIdx0 || static_cast<int>(i) == blockIdx1) {
                        continue;
                    }
                    SolveNormalScalar(manifold.contacts[i]);
                }
            }
#ifndef NDEBUG
            const float impulseContinuityMetric = std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[0] - manifold.blockSolveDebug.selectedPreNormalImpulses[0])
                + std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[1] - manifold.blockSolveDebug.selectedPreNormalImpulses[1]);
            RecordManifoldSolveTelemetry(manifold, fallbackReason, determinantOrConditionEstimate, impulseContinuityMetric);
#endif
            return;
        }

#ifndef NDEBUG
        switch (fallbackReason) {
            case BlockSolveFallbackReason::InvalidManifoldNormal:
                ++solverTelemetry_.scalarFallbackInvalidNormal;
                break;
            case BlockSolveFallbackReason::TypePolicy:
                ++solverTelemetry_.blockRejectedByTypePolicy;
                break;
            case BlockSolveFallbackReason::QualityGate:
                ++solverTelemetry_.blockRejectedByQualityOrPersistence;
                break;
            case BlockSolveFallbackReason::PersistenceGate:
                ++solverTelemetry_.scalarFallbackPersistenceGate;
                ++solverTelemetry_.blockRejectedByQualityOrPersistence;
                break;
            case BlockSolveFallbackReason::ContactNormalMismatch:
                ++solverTelemetry_.scalarFallbackNormalMismatch;
                break;
            case BlockSolveFallbackReason::MissingBlockSlots:
                ++solverTelemetry_.scalarFallbackMissingSlots;
                break;
            case BlockSolveFallbackReason::DegenerateMassMatrix:
                ++solverTelemetry_.scalarFallbackDegenerateSystem;
                break;
            case BlockSolveFallbackReason::ConditionEstimateExceeded:
                ++solverTelemetry_.scalarFallbackConditionEstimate;
                break;
            case BlockSolveFallbackReason::LcpFailure:
                ++solverTelemetry_.scalarFallbackLcpFailure;
                break;
            case BlockSolveFallbackReason::NonFiniteResult:
                ++solverTelemetry_.scalarFallbackNonFinite;
                break;
            case BlockSolveFallbackReason::Ineligible:
                break;
            case BlockSolveFallbackReason::None:
                break;
        }
        IncrementBlockSolveFallbackCounter(manifold, fallbackReason);
        if (debugBlockSolveRouting_) {
            std::fprintf(
                DebugLogStream(),
                "[minphys3d] scalar fallback pair=(%u,%u) type=%u reason=%d\n",
                manifold.a,
                manifold.b,
                static_cast<unsigned>(manifold.manifoldType),
                static_cast<int>(fallbackReason));
        }
#endif

        for (Contact& c : manifold.contacts) {
            SolveNormalScalar(c);
        }
#ifndef NDEBUG
        if (contactSolverConfig_.useFace4PointNormalBlock
            && manifold.manifoldType == 9
            && manifold.contacts.size() == 4) {
            ++solverTelemetry_.face4FallbackToScalar;
        }
        if (selectedIdx0 >= 0 && selectedIdx1 >= 0
            && static_cast<std::size_t>(std::max(selectedIdx0, selectedIdx1)) < manifold.contacts.size()) {
            manifold.blockSolveDebug.selectedPostNormalImpulses = {
                manifold.contacts[static_cast<std::size_t>(selectedIdx0)].normalImpulseSum,
                manifold.contacts[static_cast<std::size_t>(selectedIdx1)].normalImpulseSum};
        }
        const float impulseContinuityMetric = std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[0] - manifold.blockSolveDebug.selectedPreNormalImpulses[0])
            + std::abs(manifold.blockSolveDebug.selectedPostNormalImpulses[1] - manifold.blockSolveDebug.selectedPreNormalImpulses[1]);
        RecordManifoldSolveTelemetry(manifold, fallbackReason, determinantOrConditionEstimate, impulseContinuityMetric);
#endif
    }

void World::ApplyImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& ra,
        const Vec3& rb,
        const Vec3& impulse) {

        if (!a.isSleeping) {
            a.velocity -= impulse * a.invMass;
            a.angularVelocity -= invIA * Cross(ra, impulse);
        }
        if (!b.isSleeping) {
            b.velocity += impulse * b.invMass;
            b.angularVelocity += invIB * Cross(rb, impulse);
        }
    }

void World::ApplyAngularImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& angularImpulse) {

        if (!a.isSleeping) {
            a.angularVelocity -= invIA * angularImpulse;
        }
        if (!b.isSleeping) {
            b.angularVelocity += invIB * angularImpulse;
        }
    }

void World::PositionalCorrection() {

        if (contactSolverConfig_.useSplitImpulse) {
            return;
        }

        for (const Manifold& manifold : manifolds_) {
            for (const Contact& c : manifold.contacts) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];

                const float invMassSum = a.invMass + b.invMass;
                if (invMassSum <= kEpsilon) {
                    continue;
                }

                const float correctionMagnitude = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f)
                    * contactSolverConfig_.positionalCorrectionPercent / invMassSum;
                const Vec3 correction = correctionMagnitude * c.normal;

                if (!a.isSleeping) {
                    a.position -= correction * a.invMass;
                }
                if (!b.isSleeping) {
                    b.position += correction * b.invMass;
                }
            }
        }
    }

void World::ClearAccumulators() {

        for (Body& body : bodies_) {
            body.force = {0.0f, 0.0f, 0.0f};
            body.torque = {0.0f, 0.0f, 0.0f};
        }
    }

} // namespace minphys3d
