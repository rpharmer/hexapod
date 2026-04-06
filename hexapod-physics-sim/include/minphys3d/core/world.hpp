#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "minphys3d/broadphase/types.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/joints/types.hpp"
#include "minphys3d/narrowphase/dispatch.hpp"
#include "minphys3d/solver/types.hpp"

namespace minphys3d {

class World {
public:
    explicit World(Vec3 gravity = {0.0f, -9.81f, 0.0f}) : gravity_(gravity) {}

    std::uint32_t CreateBody(const Body& bodyDef) {
        Body body = bodyDef;
        body.RecomputeMassProperties();
        bodies_.push_back(body);

        BroadphaseProxy proxy;
        proxy.fatBox = ExpandAABB(body.ComputeAABB(), 0.1f);
        proxy.valid = true;
        proxies_.push_back(proxy);

        return static_cast<std::uint32_t>(bodies_.size() - 1);
    }

    Body& GetBody(std::uint32_t id) {
        return bodies_.at(id);
    }

    const Body& GetBody(std::uint32_t id) const {
        return bodies_.at(id);
    }

    std::uint32_t CreateDistanceJoint(std::uint32_t a, std::uint32_t b, const Vec3& worldAnchorA, const Vec3& worldAnchorB, float stiffness = 1.0f, float damping = 0.1f) {
        DistanceJoint j;
        j.a = a;
        j.b = b;
        j.localAnchorA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), worldAnchorA - bodies_.at(a).position);
        j.localAnchorB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), worldAnchorB - bodies_.at(b).position);
        j.restLength = Length(worldAnchorB - worldAnchorA);
        j.stiffness = stiffness;
        j.damping = damping;
        joints_.push_back(j);
        return static_cast<std::uint32_t>(joints_.size() - 1);
    }

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
        float maxMotorTorque = 0.0f) {
        HingeJoint j;
        j.a = a;
        j.b = b;
        j.localAnchorA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), worldAnchor - bodies_.at(a).position);
        j.localAnchorB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), worldAnchor - bodies_.at(b).position);
        const Vec3 axisN = Normalize(worldAxis);
        j.localAxisA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), axisN);
        j.localAxisB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), axisN);
        j.limitsEnabled = enableLimits;
        j.lowerAngle = lowerAngle;
        j.upperAngle = upperAngle;
        j.motorEnabled = enableMotor;
        j.motorSpeed = motorSpeed;
        j.maxMotorTorque = maxMotorTorque;
        hingeJoints_.push_back(j);
        return static_cast<std::uint32_t>(hingeJoints_.size() - 1);
    }

    void Step(float dt, int solverIterations = 8) {
        if (dt <= 0.0f) {
            return;
        }

        previousContacts_ = contacts_;
        previousManifolds_ = manifolds_;

        const int substeps = ComputeSubsteps(dt);
        const float subDt = dt / static_cast<float>(substeps);

        for (int stepIndex = 0; stepIndex < substeps; ++stepIndex) {
            if (stepIndex == 0) {
                previousContacts_ = contacts_;
                previousManifolds_ = manifolds_;
            }
            IntegrateForces(subDt);
            contacts_.clear();
            manifolds_.clear();
            UpdateBroadphaseProxies();
            GenerateContacts();
            BuildManifolds();
            BuildIslands();
            WarmStartContacts();
            WarmStartJoints();

            for (int i = 0; i < solverIterations; ++i) {
                SolveIslands();
            }

            IntegrateVelocities(subDt);
            ApplySphereTOIPlane(subDt);
            IntegrateOrientation(subDt);
            SolveJointPositions();
            PositionalCorrection();
            UpdateSleeping();
            ClearAccumulators();
            previousContacts_ = contacts_;
            previousManifolds_ = manifolds_;
        }
    }

    void AddForce(std::uint32_t id, const Vec3& force) {
        Body& b = bodies_.at(id);
        WakeBody(b);
        b.force += force;
    }

    void AddTorque(std::uint32_t id, const Vec3& torque) {
        Body& b = bodies_.at(id);
        WakeBody(b);
        b.torque += torque;
    }

    void AddForceAtPoint(std::uint32_t id, const Vec3& force, const Vec3& worldPoint) {
        Body& body = bodies_.at(id);
        WakeBody(body);
        body.force += force;
        const Vec3 r = worldPoint - body.position;
        body.torque += Cross(r, force);
    }

private:
    static AABB ExpandAABB(const AABB& aabb, float margin) {
        AABB out = aabb;
        out.min.x -= margin;
        out.min.y -= margin;
        out.min.z -= margin;
        out.max.x += margin;
        out.max.y += margin;
        out.max.z += margin;
        return out;
    }

    static bool Contains(const AABB& outer, const AABB& inner) {
        return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z
            && outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
    }

    void UpdateBroadphaseProxies() {
        if (proxies_.size() != bodies_.size()) {
            proxies_.resize(bodies_.size());
        }

        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            const AABB current = bodies_[i].ComputeAABB();
            if (!proxies_[i].valid) {
                proxies_[i].fatBox = ExpandAABB(current, 0.1f);
                proxies_[i].valid = true;
                continue;
            }
            if (!Contains(proxies_[i].fatBox, current)) {
                proxies_[i].fatBox = ExpandAABB(current, 0.1f);
            }
        }
    }

    int ComputeSubsteps(float dt) const {
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
            }
            const float travel = Length(body.velocity) * dt;
            maxRatio = std::max(maxRatio, travel / (characteristic * kMaxSubstepDistanceFactor));
        }
        return std::clamp(static_cast<int>(std::ceil(std::max(1.0f, maxRatio))), 1, 8);
    }

    void ApplySphereTOIPlane(float dt) {
        for (std::uint32_t i = 0; i < bodies_.size(); ++i) {
            Body& s = bodies_[i];
            if (s.shape != ShapeType::Sphere || s.invMass == 0.0f || s.isSleeping) {
                continue;
            }
            for (std::uint32_t j = 0; j < bodies_.size(); ++j) {
                const Body& p = bodies_[j];
                if (p.shape != ShapeType::Plane) {
                    continue;
                }
                const Vec3 n = Normalize(p.planeNormal);
                const float d0 = Dot(n, s.position) - p.planeOffset - s.radius;
                const float vn = Dot(n, s.velocity);
                if (d0 > 0.0f && vn < -kEpsilon) {
                    const float toi = -d0 / vn;
                    if (toi >= 0.0f && toi <= dt) {
                        s.position += s.velocity * toi;
                        const float vnNow = Dot(s.velocity, n);
                        if (vnNow < 0.0f) {
                            s.velocity -= (1.0f + s.restitution) * vnNow * n;
                        }
                        const float remaining = dt - toi;
                        s.position += s.velocity * remaining;
                    }
                }
            }
        }
    }

    static std::uint64_t MakeContactKey(std::uint32_t a, std::uint32_t b, const Vec3& point) {
        const std::uint32_t lo = std::min(a, b);
        const std::uint32_t hi = std::max(a, b);
        const auto q = [](float v) -> std::uint32_t {
            return static_cast<std::uint32_t>(std::floor((v + 1000.0f) * 20.0f));
        };
        const std::uint64_t px = static_cast<std::uint64_t>(q(point.x) & 0x7FFu);
        const std::uint64_t py = static_cast<std::uint64_t>(q(point.y) & 0x7FFu);
        const std::uint64_t pz = static_cast<std::uint64_t>(q(point.z) & 0x7FFu);
        return (static_cast<std::uint64_t>(lo) << 48)
             ^ (static_cast<std::uint64_t>(hi) << 32)
             ^ (px << 22)
             ^ (py << 11)
             ^ pz;
    }

    static void WakeBody(Body& body) {
        if (body.invMass == 0.0f) {
            return;
        }
        body.isSleeping = false;
        body.sleepCounter = 0;
    }

    static float ProjectBoxOntoAxis(const Body& box, const Vec3& axis) {
        const Vec3 u0 = Rotate(box.orientation, {1.0f, 0.0f, 0.0f});
        const Vec3 u1 = Rotate(box.orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 u2 = Rotate(box.orientation, {0.0f, 0.0f, 1.0f});
        return std::abs(Dot(axis, u0)) * box.halfExtents.x
             + std::abs(Dot(axis, u1)) * box.halfExtents.y
             + std::abs(Dot(axis, u2)) * box.halfExtents.z;
    }

    static Vec3 ClosestPointOnBox(const Body& box, const Vec3& worldPoint) {
        const Quat invQ = Conjugate(Normalize(box.orientation));
        const Vec3 local = Rotate(invQ, worldPoint - box.position);
        const Vec3 clamped = {
            std::clamp(local.x, -box.halfExtents.x, box.halfExtents.x),
            std::clamp(local.y, -box.halfExtents.y, box.halfExtents.y),
            std::clamp(local.z, -box.halfExtents.z, box.halfExtents.z),
        };
        return box.position + Rotate(box.orientation, clamped);
    }

    void IntegrateForces(float dt) {
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

    void IntegrateVelocities(float dt) {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += body.velocity * dt;
        }
    }

    void IntegrateOrientation(float dt) {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            const Quat omega{0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z};
            const Quat dq = (omega * body.orientation) * (0.5f * dt);
            body.orientation = Normalize(body.orientation + dq);
        }
    }

    void AddContact(std::uint32_t a, std::uint32_t b, const Vec3& normal, const Vec3& point, float penetration) {
        Contact c;
        c.a = a;
        c.b = b;
        c.normal = normal;
        c.point = point;
        c.penetration = penetration;
        c.key = MakeContactKey(a, b, point);

        for (const Contact& old : previousContacts_) {
            if (old.key == c.key) {
                c.normalImpulseSum = old.normalImpulseSum;
                c.tangentImpulseSum = old.tangentImpulseSum;
                break;
            }
        }

        contacts_.push_back(c);
        WakeBody(bodies_[a]);
        WakeBody(bodies_[b]);
    }

    void BuildManifolds() {
        for (const Contact& c : contacts_) {
            bool found = false;
            for (Manifold& m : manifolds_) {
                if ((m.a == c.a && m.b == c.b) || (m.a == c.b && m.b == c.a)) {
                    m.contacts.push_back(c);
                    m.normal = c.normal;
                    found = true;
                    break;
                }
            }
            if (!found) {
                Manifold m;
                m.a = c.a;
                m.b = c.b;
                m.normal = c.normal;
                m.contacts.push_back(c);
                manifolds_.push_back(m);
            }
        }

        for (Manifold& m : manifolds_) {
            for (const Manifold& old : previousManifolds_) {
                if (old.pairKey() == m.pairKey()) {
                    for (Contact& c : m.contacts) {
                        for (const Contact& oldc : old.contacts) {
                            if (oldc.key == c.key) {
                                c.normalImpulseSum = oldc.normalImpulseSum;
                                c.tangentImpulseSum = oldc.tangentImpulseSum;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    void BuildIslands() {
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
            }

            if (!island.bodies.empty()) {
                islands_.push_back(island);
            }
        }
    }

    std::vector<Pair> ComputePotentialPairs() const {
        std::vector<Pair> pairs;
        if (bodies_.empty()) {
            return pairs;
        }

        std::vector<TreeNode> nodes;
        nodes.reserve(bodies_.size() * 2);
        std::vector<std::int32_t> level;
        level.reserve(bodies_.size());

        auto mergeAABB = [](const AABB& a, const AABB& b) {
            AABB out;
            out.min = {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)};
            out.max = {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)};
            return out;
        };

        for (std::uint32_t i = 0; i < bodies_.size(); ++i) {
            TreeNode leaf;
            leaf.box = (i < proxies_.size() && proxies_[i].valid) ? proxies_[i].fatBox : ExpandAABB(bodies_[i].ComputeAABB(), 0.1f);
            leaf.bodyId = static_cast<std::int32_t>(i);
            nodes.push_back(leaf);
            level.push_back(static_cast<std::int32_t>(nodes.size() - 1));
        }

        while (level.size() > 1) {
            std::vector<std::int32_t> next;
            for (std::size_t i = 0; i < level.size(); i += 2) {
                if (i + 1 >= level.size()) {
                    next.push_back(level[i]);
                    continue;
                }
                TreeNode parent;
                parent.left = level[i];
                parent.right = level[i + 1];
                parent.box = mergeAABB(nodes[parent.left].box, nodes[parent.right].box);
                const std::int32_t parentIndex = static_cast<std::int32_t>(nodes.size());
                nodes[parent.left].parent = parentIndex;
                nodes[parent.right].parent = parentIndex;
                nodes.push_back(parent);
                next.push_back(parentIndex);
            }
            level = next;
        }

        const std::int32_t root = level.front();
        std::vector<std::pair<std::int32_t, std::int32_t>> stack;
        stack.push_back({nodes[root].left, nodes[root].right});

        while (!stack.empty()) {
            const auto [na, nb] = stack.back();
            stack.pop_back();

            if (na < 0 || nb < 0 || !Overlaps(nodes[na].box, nodes[nb].box)) {
                continue;
            }

            if (nodes[na].IsLeaf() && nodes[nb].IsLeaf()) {
                const std::uint32_t ia = static_cast<std::uint32_t>(nodes[na].bodyId);
                const std::uint32_t ib = static_cast<std::uint32_t>(nodes[nb].bodyId);
                const Body& a = bodies_[ia];
                const Body& b = bodies_[ib];
                if (ia >= ib) {
                    continue;
                }
                if (a.invMass == 0.0f && b.invMass == 0.0f) {
                    continue;
                }
                if (a.isSleeping && b.isSleeping) {
                    continue;
                }
                pairs.push_back({ia, ib});
                continue;
            }

            if (nodes[na].IsLeaf()) {
                stack.push_back({na, nodes[nb].left});
                stack.push_back({na, nodes[nb].right});
            } else if (nodes[nb].IsLeaf()) {
                stack.push_back({nodes[na].left, nb});
                stack.push_back({nodes[na].right, nb});
            } else {
                stack.push_back({nodes[na].left, nodes[nb].left});
                stack.push_back({nodes[na].left, nodes[nb].right});
                stack.push_back({nodes[na].right, nodes[nb].left});
                stack.push_back({nodes[na].right, nodes[nb].right});
            }
        }

        return pairs;
    }

    void GenerateContacts() {
        const std::vector<Pair> pairs = ComputePotentialPairs();
        for (const Pair& pair : pairs) {
            const Body& a = bodies_[pair.a];
            const Body& b = bodies_[pair.b];

            if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
                SphereSphere(pair.a, pair.b);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Capsule) {
                SphereCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Sphere) {
                SphereCapsule(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Capsule) {
                CapsuleCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Plane) {
                CapsulePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Capsule) {
                CapsulePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Box) {
                CapsuleBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Capsule) {
                CapsuleBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
                SpherePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
                SpherePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Plane) {
                BoxPlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Box) {
                BoxPlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Box) {
                SphereBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Sphere) {
                SphereBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) {
                BoxBox(pair.a, pair.b);
            }
        }
    }

    void SphereSphere(std::uint32_t ia, std::uint32_t ib) {
        const Body& a = bodies_[ia];
        const Body& b = bodies_[ib];
        const Vec3 delta = b.position - a.position;
        const float distSq = LengthSquared(delta);
        const float radiusSum = a.radius + b.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }

        const float dist = std::sqrt(std::max(distSq, kEpsilon));
        const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{1.0f, 0.0f, 0.0f};
        AddContact(ia, ib, normal, a.position + normal * a.radius, radiusSum - dist);
    }

    void SpherePlane(std::uint32_t sphereId, std::uint32_t planeId) {
        const Body& s = bodies_[sphereId];
        const Body& p = bodies_[planeId];
        const Vec3 n = Normalize(p.planeNormal);
        const float signedDistance = Dot(n, s.position) - p.planeOffset;
        if (signedDistance >= s.radius) {
            return;
        }
        AddContact(sphereId, planeId, -n, s.position - n * s.radius, s.radius - signedDistance);
    }

    void CapsulePlane(std::uint32_t capsuleId, std::uint32_t planeId) {
        const Body& c = bodies_[capsuleId];
        const Body& p = bodies_[planeId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 ends[2] = {c.position - axis * c.halfHeight, c.position + axis * c.halfHeight};
        const Vec3 n = Normalize(p.planeNormal);
        for (const Vec3& center : ends) {
            const float signedDistance = Dot(n, center) - p.planeOffset;
            if (signedDistance < c.radius) {
                AddContact(capsuleId, planeId, -n, center - n * c.radius, c.radius - signedDistance);
            }
        }
    }

    void SphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId) {
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
        AddContact(sphereId, capsuleId, normal, s.position + normal * s.radius, radiusSum - dist);
    }

    void CapsuleCapsule(std::uint32_t aId, std::uint32_t bId) {
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
        const Vec3 r = p1 - p2;
        const float aLen = Dot(d1, d1);
        const float eLen = Dot(d2, d2);
        const float f = Dot(d2, r);

        float s = 0.0f;
        float t = 0.0f;

        if (aLen <= kEpsilon && eLen <= kEpsilon) {
            s = 0.0f;
            t = 0.0f;
        } else if (aLen <= kEpsilon) {
            s = 0.0f;
            t = std::clamp(f / eLen, 0.0f, 1.0f);
        } else {
            const float cTerm = Dot(d1, r);
            if (eLen <= kEpsilon) {
                t = 0.0f;
                s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
            } else {
                const float bDot = Dot(d1, d2);
                const float denom2 = aLen * eLen - bDot * bDot;
                if (std::abs(denom2) > kEpsilon) {
                    s = std::clamp((bDot * f - cTerm * eLen) / denom2, 0.0f, 1.0f);
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
            }
        }

        const Vec3 c1 = p1 + d1 * s;
        const Vec3 c2 = p2 + d2 * t;
        const Vec3 delta = c2 - c1;
        const float distSq = LengthSquared(delta);
        const float radiusSum = a.radius + b.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }

        const float dist = std::sqrt(std::max(distSq, kEpsilon));
        const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{0.0f, 1.0f, 0.0f};
        const Vec3 point = c1 + normal * a.radius;
        AddContact(aId, bId, normal, point, radiusSum - dist);
    }

    void CapsuleBox(std::uint32_t capsuleId, std::uint32_t boxId) {
        const Body& c = bodies_[capsuleId];
        const Body& b = bodies_[boxId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 segAWorld = c.position - axis * c.halfHeight;
        const Vec3 segBWorld = c.position + axis * c.halfHeight;

        const Quat invBoxOrientation = Conjugate(Normalize(b.orientation));
        const Vec3 segA = Rotate(invBoxOrientation, segAWorld - b.position);
        const Vec3 segB = Rotate(invBoxOrientation, segBWorld - b.position);
        const Vec3 d = segB - segA;

        auto clampToBox = [&](const Vec3& p) {
            return Vec3{
                std::clamp(p.x, -b.halfExtents.x, b.halfExtents.x),
                std::clamp(p.y, -b.halfExtents.y, b.halfExtents.y),
                std::clamp(p.z, -b.halfExtents.z, b.halfExtents.z),
            };
        };

        float bestDistSq = std::numeric_limits<float>::infinity();
        Vec3 bestSeg{};
        Vec3 bestBox{};

        auto testPoint = [&](float t) {
            const Vec3 p = segA + d * t;
            const Vec3 q = clampToBox(p);
            const float distSq = LengthSquared(p - q);
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestSeg = p;
                bestBox = q;
            }
        };

        testPoint(0.0f);
        testPoint(1.0f);
        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            const float p0 = (axisIndex == 0) ? segA.x : (axisIndex == 1 ? segA.y : segA.z);
            const float vd = (axisIndex == 0) ? d.x : (axisIndex == 1 ? d.y : d.z);
            const float minB = -(axisIndex == 0 ? b.halfExtents.x : (axisIndex == 1 ? b.halfExtents.y : b.halfExtents.z));
            const float maxB =  (axisIndex == 0 ? b.halfExtents.x : (axisIndex == 1 ? b.halfExtents.y : b.halfExtents.z));
            if (std::abs(vd) <= kEpsilon) {
                continue;
            }
            const float t1 = (minB - p0) / vd;
            const float t2 = (maxB - p0) / vd;
            if (t1 >= 0.0f && t1 <= 1.0f) testPoint(t1);
            if (t2 >= 0.0f && t2 <= 1.0f) testPoint(t2);
        }

        if (bestDistSq > c.radius * c.radius) {
            return;
        }

        const float dist = std::sqrt(std::max(bestDistSq, kEpsilon));
        const Vec3 normalLocal = (dist > kEpsilon) ? ((bestSeg - bestBox) / dist) : Vec3{0.0f, 1.0f, 0.0f};
        const Vec3 normalWorld = Rotate(b.orientation, normalLocal);
        const Vec3 pointWorld = b.position + Rotate(b.orientation, bestBox);
        AddContact(capsuleId, boxId, -normalWorld, pointWorld, c.radius - dist);
    }

    void BoxPlane(std::uint32_t boxId, std::uint32_t planeId) {
        const Body& box = bodies_[boxId];
        const Body& plane = bodies_[planeId];
        const Vec3 n = Normalize(plane.planeNormal);

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
                        AddContact(boxId, planeId, -n, worldPoint, -signedDistance);
                        ++added;
                        if (added >= 4) {
                            return;
                        }
                    }
                }
            }
        }
    }

    void SphereBox(std::uint32_t sphereId, std::uint32_t boxId) {
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

        AddContact(sphereId, boxId, -normalWorld, pointWorld, penetration);
    }

    void BoxBox(std::uint32_t aId, std::uint32_t bId) {
        const Body& a = bodies_[aId];
        const Body& b = bodies_[bId];

        const Vec3 aAxes[3] = {
            Rotate(a.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(a.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(a.orientation, {0.0f, 0.0f, 1.0f}),
        };
        const Vec3 bAxes[3] = {
            Rotate(b.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(b.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(b.orientation, {0.0f, 0.0f, 1.0f}),
        };

        struct AxisCandidate {
            Vec3 axis{};
            int type = 0; // 0 = face A, 1 = face B, 2 = edge-edge
            int indexA = -1;
            int indexB = -1;
        };

        AxisCandidate candidates[15];
        int axisCount = 0;
        for (int i = 0; i < 3; ++i) candidates[axisCount++] = {aAxes[i], 0, i, -1};
        for (int i = 0; i < 3; ++i) candidates[axisCount++] = {bAxes[i], 1, -1, i};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                candidates[axisCount++] = {Cross(aAxes[i], bAxes[j]), 2, i, j};
            }
        }

        const Vec3 centerDelta = b.position - a.position;
        float bestOverlap = std::numeric_limits<float>::infinity();
        AxisCandidate best{};

        for (int i = 0; i < axisCount; ++i) {
            Vec3 axis = candidates[i].axis;
            const float lenSq = LengthSquared(axis);
            if (lenSq <= 1e-8f) {
                continue;
            }
            axis = axis / std::sqrt(lenSq);

            const float ra = ProjectBoxOntoAxis(a, axis);
            const float rb = ProjectBoxOntoAxis(b, axis);
            const float distance = std::abs(Dot(centerDelta, axis));
            const float overlap = ra + rb - distance;
            if (overlap < 0.0f) {
                return;
            }
            if (overlap < bestOverlap) {
                bestOverlap = overlap;
                best = candidates[i];
                best.axis = axis;
                if (Dot(centerDelta, best.axis) < 0.0f) {
                    best.axis = -best.axis;
                }
            }
        }

        if (bestOverlap == std::numeric_limits<float>::infinity()) {
            return;
        }

        auto localFaceVertices = [](const Vec3& he, int axis, float sign) {
            std::vector<Vec3> verts;
            verts.reserve(4);
            if (axis == 0) {
                verts.push_back({sign * he.x, -he.y, -he.z});
                verts.push_back({sign * he.x,  he.y, -he.z});
                verts.push_back({sign * he.x,  he.y,  he.z});
                verts.push_back({sign * he.x, -he.y,  he.z});
            } else if (axis == 1) {
                verts.push_back({-he.x, sign * he.y, -he.z});
                verts.push_back({ he.x, sign * he.y, -he.z});
                verts.push_back({ he.x, sign * he.y,  he.z});
                verts.push_back({-he.x, sign * he.y,  he.z});
            } else {
                verts.push_back({-he.x, -he.y, sign * he.z});
                verts.push_back({ he.x, -he.y, sign * he.z});
                verts.push_back({ he.x,  he.y, sign * he.z});
                verts.push_back({-he.x,  he.y, sign * he.z});
            }
            return verts;
        };

        auto worldFromLocal = [](const Body& body, const Vec3& p) {
            return body.position + Rotate(body.orientation, p);
        };

        auto clipPolygonAgainstPlane = [](const std::vector<Vec3>& poly, const Vec3& n, float d) {
            std::vector<Vec3> out;
            if (poly.empty()) {
                return out;
            }
            for (std::size_t i = 0; i < poly.size(); ++i) {
                const Vec3 a = poly[i];
                const Vec3 b = poly[(i + 1) % poly.size()];
                const float da = Dot(n, a) - d;
                const float db = Dot(n, b) - d;
                const bool ina = da <= 0.0f;
                const bool inb = db <= 0.0f;

                if (ina && inb) {
                    out.push_back(b);
                } else if (ina && !inb) {
                    const float t = da / (da - db + kEpsilon);
                    out.push_back(a + (b - a) * t);
                } else if (!ina && inb) {
                    const float t = da / (da - db + kEpsilon);
                    out.push_back(a + (b - a) * t);
                    out.push_back(b);
                }
            }
            return out;
        };

        auto addUniquePoint = [&](std::vector<Vec3>& pts, const Vec3& p) {
            for (const Vec3& e : pts) {
                if (LengthSquared(e - p) < 1e-4f) {
                    return;
                }
            }
            pts.push_back(p);
        };

        if (best.type == 0 || best.type == 1) {
            const bool aReference = (best.type == 0);
            const Body& ref = aReference ? a : b;
            const Body& inc = aReference ? b : a;
            const int refAxis = aReference ? best.indexA : best.indexB;

            Vec3 refNormal = best.axis;
            if (!aReference) {
                refNormal = -refNormal;
            }

            const Vec3 refBasis[3] = {
                Rotate(ref.orientation, {1.0f, 0.0f, 0.0f}),
                Rotate(ref.orientation, {0.0f, 1.0f, 0.0f}),
                Rotate(ref.orientation, {0.0f, 0.0f, 1.0f}),
            };
            const Vec3 incBasis[3] = {
                Rotate(inc.orientation, {1.0f, 0.0f, 0.0f}),
                Rotate(inc.orientation, {0.0f, 1.0f, 0.0f}),
                Rotate(inc.orientation, {0.0f, 0.0f, 1.0f}),
            };

            Vec3 refAxisDir = refBasis[refAxis];
            float refSign = 1.0f;
            if (Dot(refAxisDir, refNormal) < 0.0f) {
                refAxisDir = -refAxisDir;
                refSign = -1.0f;
            }

            int incidentAxis = 0;
            float minDot = std::numeric_limits<float>::infinity();
            for (int i = 0; i < 3; ++i) {
                const float d = Dot(incBasis[i], refNormal);
                if (d < minDot) {
                    minDot = d;
                    incidentAxis = i;
                }
            }
            float incSign = (Dot(incBasis[incidentAxis], refNormal) > 0.0f) ? -1.0f : 1.0f;

            std::vector<Vec3> poly = localFaceVertices(inc.halfExtents, incidentAxis, incSign);
            for (Vec3& p : poly) {
                p = worldFromLocal(inc, p);
            }

            const int u = (refAxis + 1) % 3;
            const int v = (refAxis + 2) % 3;
            const Vec3 sideU = refBasis[u];
            const Vec3 sideV = refBasis[v];
            const float limitU = (u == 0 ? ref.halfExtents.x : (u == 1 ? ref.halfExtents.y : ref.halfExtents.z));
            const float limitV = (v == 0 ? ref.halfExtents.x : (v == 1 ? ref.halfExtents.y : ref.halfExtents.z));

            poly = clipPolygonAgainstPlane(poly,  sideU, Dot(sideU, ref.position) + limitU);
            poly = clipPolygonAgainstPlane(poly, -sideU, Dot(-sideU, ref.position) + limitU);
            poly = clipPolygonAgainstPlane(poly,  sideV, Dot(sideV, ref.position) + limitV);
            poly = clipPolygonAgainstPlane(poly, -sideV, Dot(-sideV, ref.position) + limitV);

            Vec3 localPlanePoint{};
            if (refAxis == 0) localPlanePoint = {refSign * ref.halfExtents.x, 0.0f, 0.0f};
            if (refAxis == 1) localPlanePoint = {0.0f, refSign * ref.halfExtents.y, 0.0f};
            if (refAxis == 2) localPlanePoint = {0.0f, 0.0f, refSign * ref.halfExtents.z};
            const Vec3 planePoint = worldFromLocal(ref, localPlanePoint);

            std::vector<Vec3> contacts;
            for (const Vec3& p : poly) {
                const float depth = Dot(planePoint - p, refNormal);
                if (depth >= -0.02f) {
                    addUniquePoint(contacts, p + refNormal * std::min(depth, 0.0f));
                }
            }

            if (contacts.empty()) {
                const Vec3 pointA = ClosestPointOnBox(a, b.position);
                const Vec3 pointB = ClosestPointOnBox(b, a.position);
                contacts.push_back(0.5f * (pointA + pointB));
            }

            const Vec3 normalAB = aReference ? best.axis : -best.axis;
            for (std::size_t i = 0; i < std::min<std::size_t>(4, contacts.size()); ++i) {
                AddContact(aId, bId, normalAB, contacts[i], bestOverlap);
            }
            return;
        }

        const Vec3 pointA = ClosestPointOnBox(a, b.position);
        const Vec3 pointB = ClosestPointOnBox(b, a.position);
        AddContact(aId, bId, best.axis, 0.5f * (pointA + pointB), bestOverlap);
    }

    void WarmStartContacts() {
        for (Manifold& m : manifolds_) {
            for (Contact& c : m.contacts) {
                if (c.normalImpulseSum == 0.0f && c.tangentImpulseSum == 0.0f) {
                    continue;
                }

                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                const Mat3 invIA = a.InvInertiaWorld();
                const Mat3 invIB = b.InvInertiaWorld();
                const Vec3 ra = c.point - a.position;
                const Vec3 rb = c.point - b.position;

                Vec3 tangent{0.0f, 0.0f, 0.0f};
                const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
                const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
                const Vec3 rv = vb - va;
                Vec3 trial = rv - Dot(rv, c.normal) * c.normal;
                if (LengthSquared(trial) > kEpsilon) {
                    tangent = Normalize(trial);
                }

                const Vec3 impulse = c.normalImpulseSum * c.normal + c.tangentImpulseSum * tangent;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
            }
        }
    }

    void WarmStartJoints() {
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
    }

    void SolveContactsInManifold(Manifold& manifold) {
        for (Contact& c : manifold.contacts) {
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

            const float restitution = std::min(a.restitution, b.restitution);
            const Vec3 raCrossN = Cross(ra, c.normal);
            const Vec3 rbCrossN = Cross(rb, c.normal);
            const float angularTermA = Dot(raCrossN, invIA * raCrossN);
            const float angularTermB = Dot(rbCrossN, invIB * rbCrossN);
            const float normalMass = a.invMass + b.invMass + angularTermA + angularTermB;
            if (normalMass <= kEpsilon) {
                continue;
            }

            float lambdaN = -(1.0f + restitution) * separatingVelocity / normalMass;
            const float oldNormalImpulse = c.normalImpulseSum;
            c.normalImpulseSum = std::max(0.0f, c.normalImpulseSum + lambdaN);
            lambdaN = c.normalImpulseSum - oldNormalImpulse;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambdaN * c.normal);

            const Vec3 va2 = a.velocity + Cross(a.angularVelocity, ra);
            const Vec3 vb2 = b.velocity + Cross(b.angularVelocity, rb);
            const Vec3 rv2 = vb2 - va2;
            Vec3 tangent = rv2 - Dot(rv2, c.normal) * c.normal;
            const float tangentLenSq = LengthSquared(tangent);
            if (tangentLenSq <= kEpsilon) {
                continue;
            }
            tangent = tangent / std::sqrt(tangentLenSq);

            const Vec3 raCrossT = Cross(ra, tangent);
            const Vec3 rbCrossT = Cross(rb, tangent);
            const float tangentMass = a.invMass + b.invMass
                + Dot(raCrossT, invIA * raCrossT)
                + Dot(rbCrossT, invIB * rbCrossT);
            if (tangentMass <= kEpsilon) {
                continue;
            }

            float lambdaT = -Dot(rv2, tangent) / tangentMass;
            const float muS = 0.5f * (a.staticFriction + b.staticFriction);
            const float maxFriction = muS * c.normalImpulseSum;
            const float oldTangentImpulse = c.tangentImpulseSum;
            c.tangentImpulseSum = std::clamp(c.tangentImpulseSum + lambdaT, -maxFriction, maxFriction);
            lambdaT = c.tangentImpulseSum - oldTangentImpulse;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambdaT * tangent);
        }
    }

    void SolveDistanceJoint(DistanceJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
        const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
        const Vec3 pa = a.position + ra;
        const Vec3 pb = b.position + rb;
        const Vec3 delta = pb - pa;
        const float len = Length(delta);
        if (len <= kEpsilon) {
            return;
        }

        const Vec3 n = delta / len;
        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const float relVel = Dot(vb - va, n);
        const float error = len - j.restLength;

        const Vec3 raCrossN = Cross(ra, n);
        const Vec3 rbCrossN = Cross(rb, n);
        const float effMass = a.invMass + b.invMass
            + Dot(raCrossN, invIA * raCrossN)
            + Dot(rbCrossN, invIB * rbCrossN);
        if (effMass <= kEpsilon) {
            return;
        }

        const float bias = j.stiffness * error + j.damping * relVel;
        const float lambda = -bias / effMass;
        j.impulseSum += lambda;
        ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
    }

    void SolveHingeJoint(HingeJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
        const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
        const Vec3 pa = a.position + ra;
        const Vec3 pb = b.position + rb;
        const Vec3 error = pb - pa;
        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const Vec3 relVel = vb - va;

        const Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        float* impulseSums[3] = {&j.impulseX, &j.impulseY, &j.impulseZ};

        for (int i = 0; i < 3; ++i) {
            const Vec3 n = axes[i];
            const Vec3 raCrossN = Cross(ra, n);
            const Vec3 rbCrossN = Cross(rb, n);
            const float effMass = a.invMass + b.invMass
                + Dot(raCrossN, invIA * raCrossN)
                + Dot(rbCrossN, invIB * rbCrossN);
            if (effMass <= kEpsilon) {
                continue;
            }

            const float bias = 0.2f * Dot(error, n) + 0.1f * Dot(relVel, n);
            const float lambda = -bias / effMass;
            *impulseSums[i] += lambda;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
        }

        const Vec3 axisA = Normalize(Rotate(a.orientation, j.localAxisA));
        const Vec3 axisB = Normalize(Rotate(b.orientation, j.localAxisB));
        Vec3 t1 = Cross(axisA, {1.0f, 0.0f, 0.0f});
        if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axisA, {0.0f, 0.0f, 1.0f});
        t1 = Normalize(t1);
        const Vec3 t2 = Normalize(Cross(axisA, t1));

        const Vec3 angularError = Cross(axisA, axisB);
        const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;

        const Vec3 angAxes[2] = {t1, t2};
        float* angImpulseSums[2] = {&j.angularImpulse1, &j.angularImpulse2};
        for (int i = 0; i < 2; ++i) {
            const Vec3 n = angAxes[i];
            const float effMass = Dot(n, invIA * n) + Dot(n, invIB * n);
            if (effMass <= kEpsilon) {
                continue;
            }
            const float bias = 0.2f * Dot(angularError, n) + 0.1f * Dot(relAngVel, n);
            const float lambda = -bias / effMass;
            *angImpulseSums[i] += lambda;
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * n);
        }

        float hingeAngle = std::atan2(Dot(Cross(axisA, axisB), t1), Dot(axisA, axisB));
        if (j.limitsEnabled) {
            float limitError = 0.0f;
            if (hingeAngle < j.lowerAngle) {
                limitError = hingeAngle - j.lowerAngle;
            } else if (hingeAngle > j.upperAngle) {
                limitError = hingeAngle - j.upperAngle;
            }
            if (std::abs(limitError) > kEpsilon) {
                const float effMass = Dot(axisA, invIA * axisA) + Dot(axisA, invIB * axisA);
                if (effMass > kEpsilon) {
                    const float lambda = -(0.2f * limitError + 0.05f * Dot(relAngVel, axisA)) / effMass;
                    ApplyAngularImpulse(a, b, invIA, invIB, lambda * axisA);
                }
            }
        }

        if (j.motorEnabled) {
            const float effMass = Dot(axisA, invIA * axisA) + Dot(axisA, invIB * axisA);
            if (effMass > kEpsilon) {
                float lambda = -(Dot(relAngVel, axisA) - j.motorSpeed) / effMass;
                const float oldImpulse = j.motorImpulseSum;
                j.motorImpulseSum = std::clamp(j.motorImpulseSum + lambda, -j.maxMotorTorque, j.maxMotorTorque);
                lambda = j.motorImpulseSum - oldImpulse;
                ApplyAngularImpulse(a, b, invIA, invIB, lambda * axisA);
            }
        }
    }

    void SolveIslands() {
        for (const Island& island : islands_) {
            for (std::size_t mi : island.manifolds) {
                SolveContactsInManifold(manifolds_[mi]);
            }
            for (std::size_t ji : island.joints) {
                SolveDistanceJoint(joints_[ji]);
            }
            for (std::size_t hi : island.hinges) {
                SolveHingeJoint(hingeJoints_[hi]);
            }
        }
    }

    void SolveJointPositions() {
        for (DistanceJoint& j : joints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (a.invMass + b.invMass <= kEpsilon) {
                continue;
            }

            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 pa = a.position + ra;
            const Vec3 pb = b.position + rb;
            const Vec3 delta = pb - pa;
            const float len = Length(delta);
            if (len <= kEpsilon) {
                continue;
            }

            const Vec3 n = delta / len;
            const float error = len - j.restLength;
            const float invMassSum = a.invMass + b.invMass;
            if (invMassSum <= kEpsilon) {
                continue;
            }
            const Vec3 correction = (0.15f * error / invMassSum) * n;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }

        for (HingeJoint& j : hingeJoints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (a.invMass + b.invMass <= kEpsilon) {
                continue;
            }

            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 pa = a.position + ra;
            const Vec3 pb = b.position + rb;
            const Vec3 error = pb - pa;
            const float invMassSum = a.invMass + b.invMass;
            const Vec3 correction = (0.2f / std::max(invMassSum, kEpsilon)) * error;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }
    }

    void ApplyImpulse(
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

    void ApplyAngularImpulse(
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

    void PositionalCorrection() {
        constexpr float percent = 0.8f;
        constexpr float slop = 0.01f;

        for (const Manifold& manifold : manifolds_) {
            for (const Contact& c : manifold.contacts) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];

                const float invMassSum = a.invMass + b.invMass;
                if (invMassSum <= kEpsilon) {
                    continue;
                }

                const float correctionMagnitude = std::max(c.penetration - slop, 0.0f) * percent / invMassSum;
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

    void UpdateSleeping() {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f) {
                continue;
            }

            const float linearSpeedSq = LengthSquared(body.velocity);
            const float angularSpeedSq = LengthSquared(body.angularVelocity);
            const bool nearlyStill = linearSpeedSq < (kSleepLinearThreshold * kSleepLinearThreshold)
                                  && angularSpeedSq < (kSleepAngularThreshold * kSleepAngularThreshold);

            if (nearlyStill) {
                ++body.sleepCounter;
                if (body.sleepCounter >= kSleepFramesThreshold) {
                    body.isSleeping = true;
                    body.velocity = {0.0f, 0.0f, 0.0f};
                    body.angularVelocity = {0.0f, 0.0f, 0.0f};
                }
            } else {
                body.isSleeping = false;
                body.sleepCounter = 0;
            }
        }
    }

    void ClearAccumulators() {
        for (Body& body : bodies_) {
            body.force = {0.0f, 0.0f, 0.0f};
            body.torque = {0.0f, 0.0f, 0.0f};
        }
    }

private:
    Vec3 gravity_{};
    std::vector<Body> bodies_;
    std::vector<BroadphaseProxy> proxies_;
    std::vector<Contact> contacts_;
    std::vector<Contact> previousContacts_;
    std::vector<Manifold> manifolds_;
    std::vector<Manifold> previousManifolds_;
    std::vector<Island> islands_;
    std::vector<DistanceJoint> joints_;
    std::vector<HingeJoint> hingeJoints_;
};


} // namespace minphys3d
