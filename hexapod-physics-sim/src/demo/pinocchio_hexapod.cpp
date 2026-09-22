#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/constraint-cholesky.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/delassus-operator.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/solvers/admm-solver.hpp>
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/multibody/joint.hpp>

#include "hexapod_dynamics_constants.hpp"
#include "contact_warm_start.hpp"
#include "servo_pd_request.hpp"

namespace minphys3d::demo {
namespace {

Eigen::Vector3d ToEigen(const Vec3& v) {
    return {v.x, v.y, v.z};
}

double servoTorqueScaleFromEnv() {
    const char* value = std::getenv("HEXAPOD_SERVO_TORQUE_SCALE");
    if (value == nullptr || value[0] == '\0') {
        return 1.0;
    }
    const double parsed = std::atof(value);
    return parsed > 0.0 ? parsed : 1.0;
}

Vec3 FromEigen(const Eigen::Vector3d& v) {
    return {v.x(), v.y(), v.z()};
}

Eigen::Matrix3d ToEigen(const Mat3& m) {
    Eigen::Matrix3d out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = m.m[row][col];
        }
    }
    return out;
}

Eigen::Matrix3d ToEigenRotation(const Quat& q) {
    return ToEigen(RotationMatrix(q));
}

pinocchio::SE3 BodyPose(const Body& body) {
    return {ToEigenRotation(body.orientation), ToEigen(body.position)};
}

pinocchio::Inertia BodyInertia(const Body& body) {
    Mat3 inertia_local{};
    if (!InvertMat3(body.invInertiaLocal, inertia_local)) {
        throw std::runtime_error("dynamic body has singular local inertia");
    }
    return {body.mass, Eigen::Vector3d::Zero(), ToEigen(inertia_local)};
}

double BoundedEnvDouble(
    const char* name, double fallback, double minimum, double maximum) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end != value && *end == '\0' && std::isfinite(parsed)
            && parsed >= minimum && parsed <= maximum
        ? parsed
        : fallback;
}

ProximalSpeedLimitFrame SpeedLimitFrameFromName(const std::string& frameName) {
    if (frameName == "chassis_body") {
        return ProximalSpeedLimitFrame::Chassis;
    }
    if (frameName.find("_coxa") != std::string::npos) {
        return ProximalSpeedLimitFrame::Coxa;
    }
    if (frameName.find("_femur") != std::string::npos) {
        return ProximalSpeedLimitFrame::Femur;
    }
    if (frameName.find("_tibia") != std::string::npos) {
        return ProximalSpeedLimitFrame::Tibia;
    }
    return ProximalSpeedLimitFrame::None;
}

std::optional<std::size_t> SpeedLimitLegIndexFromName(const std::string& frameName) {
    if (frameName.compare(0, 4, "leg_") != 0) {
        return std::nullopt;
    }
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(frameName.c_str() + 4, &end, 10);
    if (end == frameName.c_str() + 4 || parsed > 5) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(parsed);
}

Quat FromEigenRotation(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond q(rotation);
    return Normalize(Quat{q.w(), q.x(), q.y(), q.z()});
}

bool IsFinite(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

bool BodyStateWithinBounds(const Body& body, const ProximalSolverSettings& settings) {
    const double linear = Length(body.velocity), angular = Length(body.angularVelocity);
    const auto& rotation = body.orientation;
    const double quaternionNormSquared = rotation.w * rotation.w + rotation.x * rotation.x
        + rotation.y * rotation.y + rotation.z * rotation.z;
    return std::isfinite(body.position.x) && std::isfinite(body.position.y) && std::isfinite(body.position.z)
        && std::isfinite(quaternionNormSquared) && std::abs(quaternionNormSquared - 1.0) <= 1e-6
        && std::isfinite(linear) && std::isfinite(angular)
        && linear <= settings.maxLinearSpeed && angular <= settings.maxAngularSpeed;
}

Eigen::Vector3d ProjectCoulombImpulse(const Eigen::Vector3d& impulse, double friction) {
    const double mu = std::max(0.0, friction);
    const double tangentNorm = impulse.head<2>().norm();
    const double normal = impulse[2];
    if (normal >= 0.0 && tangentNorm <= mu * normal) {
        return impulse;
    }
    if (mu <= 0.0 || tangentNorm <= -normal / mu) {
        return Eigen::Vector3d::Zero();
    }

    const double projectedNormal = (mu * tangentNorm + normal) / (mu * mu + 1.0);
    Eigen::Vector3d projected{};
    projected[2] = std::max(0.0, projectedNormal);
    if (tangentNorm > 0.0) {
        projected.head<2>() = impulse.head<2>()
            * (mu * projected[2] / tangentNorm);
    }
    return projected;
}

std::uint64_t PersistentContactId(const Manifold& manifold, const Contact& contact) {
    if (manifold.contacts.size() == 1U) {
        return manifold.pairKey();
    }
    std::uint64_t value = manifold.pairKey();
    value ^= contact.key + 0x9e3779b97f4a7c15ULL + (value << 6U) + (value >> 2U);
    return value;
}

std::uint64_t ContactOrderKey(std::uint64_t value, const std::uint64_t seed) {
    if (seed == 0) {
        return value;
    }
    value += seed + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

Eigen::Matrix3d ContactFrameRotation(const Vec3& inputNormal) {
    const Vec3 normal = Normalize(inputNormal);
    Vec3 tangent0{};
    Vec3 tangent1{};
    if (!World::ComputeStableTangentFrame(normal, Vec3{}, tangent0, tangent1)) {
        tangent0 = std::abs(normal.y) < 0.9 ? Normalize(Cross({0.0, 1.0, 0.0}, normal))
                                           : Normalize(Cross({1.0, 0.0, 0.0}, normal));
        tangent1 = Normalize(Cross(normal, tangent0));
    }
    Eigen::Matrix3d rotation;
    rotation.col(0) = ToEigen(tangent0);
    rotation.col(1) = ToEigen(tangent1);
    rotation.col(2) = ToEigen(normal);
    return rotation;
}

Eigen::Vector3d TransportContactFrameVector(
    const Eigen::Matrix3d& oldFrame,
    const Eigen::Matrix3d& newFrame,
    const Eigen::Vector3d& contactVector) {
    return newFrame.transpose() * (oldFrame * contactVector);
}

double ContactFrameRotationAngleRad(
    const Eigen::Matrix3d& oldFrame, const Eigen::Matrix3d& newFrame) {
    const double cosine = std::clamp(
        0.5 * ((oldFrame.transpose() * newFrame).trace() - 1.0), -1.0, 1.0);
    return std::acos(cosine);
}

std::string JsonEscape(const std::string& value) {
    std::string out = "\"";
    for (const char ch : value) {
        if (ch == '"' || ch == '\\') {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

void WriteJsonNumberArray(std::ostream& out, const std::vector<double>& values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        const double value = std::isfinite(values[i]) ? values[i] : 0.0;
        out << value;
    }
    out << ']';
}

template <std::size_t N>
void WriteJsonNumberArray(std::ostream& out, const std::array<double, N>& values) {
    out << '[';
    for (std::size_t i = 0; i < N; ++i) {
        if (i != 0) {
            out << ',';
        }
        const double value = std::isfinite(values[i]) ? values[i] : 0.0;
        out << value;
    }
    out << ']';
}

void SkipJsonWhitespace(const std::string& text, std::size_t& i) {
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
        ++i;
    }
}

bool FindJsonKeyInRange(
    const std::string& text,
    const char* key,
    std::size_t from,
    std::size_t to,
    std::size_t& valuePos) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    valuePos = pos + needle.size();
    return true;
}

bool ParseJsonNumberArray(const std::string& text, std::size_t& i, std::vector<double>& out) {
    SkipJsonWhitespace(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipJsonWhitespace(text, i);
    if (i < text.size() && text[i] == ']') {
        ++i;
        return true;
    }
    while (i < text.size()) {
        SkipJsonWhitespace(text, i);
        if (i < text.size() && text[i] == '[') {
            std::vector<double> nested;
            if (!ParseJsonNumberArray(text, i, nested)) {
                return false;
            }
            out.insert(out.end(), nested.begin(), nested.end());
        } else {
            char* end = nullptr;
            const double value = std::strtod(text.c_str() + i, &end);
            if (end == text.c_str() + i) {
                return false;
            }
            out.push_back(value);
            i = static_cast<std::size_t>(end - text.c_str());
        }
        SkipJsonWhitespace(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
            continue;
        }
        if (i < text.size() && text[i] == ']') {
            ++i;
            return true;
        }
        return false;
    }
    return false;
}

bool ParseJsonBool(const std::string& text, std::size_t i, bool& out) {
    SkipJsonWhitespace(text, i);
    if (text.compare(i, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (text.compare(i, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

bool ParseJsonNumber(const std::string& text, std::size_t i, double& out) {
    SkipJsonWhitespace(text, i);
    char* end = nullptr;
    const double value = std::strtod(text.c_str() + i, &end);
    if (end == text.c_str() + i || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

bool ExtractJsonQuoted(
    const std::string& text, const char* key, std::size_t from, std::size_t to, std::string& out) {
    std::size_t i = 0;
    if (!FindJsonKeyInRange(text, key, from, to, i)) {
        return false;
    }
    SkipJsonWhitespace(text, i);
    if (i >= text.size() || text[i] != '"') {
        return false;
    }
    ++i;
    out.clear();
    while (i < text.size() && text[i] != '"') {
        if (text[i] == '\\' && i + 1 < text.size()) {
            out.push_back(text[i + 1]);
            i += 2;
            continue;
        }
        out.push_back(text[i++]);
    }
    return i < text.size();
}

template <std::size_t N>
bool CopyJsonArray(const std::vector<double>& src, std::array<double, N>& dst) {
    if (src.size() != N) {
        return false;
    }
    std::copy(src.begin(), src.end(), dst.begin());
    return true;
}

bool LoadEntireFile(const char* path, std::string& out) {
    if (path == nullptr || path[0] == '\0') {
        return false;
    }
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return !out.empty();
}

constexpr double kMaxStanceInertiaScale = 1.5;
constexpr double kMaxReducedContactInertiaScale = 2.625;
constexpr double kStanceContactRegularization = 1.0e-8;
constexpr double kMinConstrainedInverseInertia = 1.0e-12;
// Six-foot CRBA calibration underestimates the effective load once a static
// pose unloads one or more legs.  A delayed, ramped correction preserves the
// 25 rad/s servo model without changing stall torque or moving-gait dynamics.
constexpr double kStaticReducedSupportGainScale = 1.85;
constexpr double kStaticReducedSupportDwellS = 0.250;
constexpr double kStaticReducedSupportRampS = 0.100;
constexpr double kStaticReducedSupportLegTargetDeltaRad = 0.10;
// Recompute the spawn CRBA on the current load-bearing contact set and blend
// the PD inertia over this window. Do not use a dimensionless moving-gait gain.
// Retarget at most once per physics step so WAVE cannot hitch the serve thread.
constexpr double kContactInertiaRampS = 0.050;
constexpr double kLoadBearingNormalImpulseFraction = 0.02;
// Represent the MG996R geartrain/rotor inertia at each output joint. Without
// armature the scalar reflected inertia used to tune the 25 rad/s PD law is
// absent from ABA's plant, leaving very light coupled modes that can reverse
// by tens of rad/s in one substep.
constexpr double kServoArmatureInertiaScale = 0.25;
// SpeedLimit is a velocity guard; dt/2 does not reduce WORLD_ALIGNED ω.
// Retry that class at half proportional drive; the explicit production motor
// retains nominal damping so braking is not weakened near the guard. The
// resulting vNew still has to pass the all-body cap. Do not skip link frames.
constexpr double kSpeedLimitRetryGainScale = 0.5;

Vec3 TibiaFootWorldPosition(const Body& tibia) {
    for (const CompoundChild& child : tibia.compoundChildren) {
        if (child.shape == ShapeType::Sphere) {
            return tibia.position + Rotate(tibia.orientation, child.localPosition);
        }
    }
    return tibia.position;
}

bool AssignStanceLoadedServoInertias(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const World& world,
    const HexapodSceneObjects& scene,
    const Eigen::Map<const Eigen::VectorXd>& q,
    const std::array<pinocchio::JointIndex, 18>& wireJoints,
    const Eigen::MatrixXd& unconstrainedMass,
    std::array<double, 18>& unconstrainedInertias,
    std::array<double, 18>& nominalInertias,
    const std::array<bool, 6>* loadBearingLegs = nullptr,
    double maxInertiaScale = kMaxStanceInertiaScale) {
    const Eigen::Index nv = model.nv;
    if (unconstrainedMass.rows() != nv || unconstrainedMass.cols() != nv) {
        return false;
    }

    for (std::size_t i = 0; i < wireJoints.size(); ++i) {
        const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
        const double inertia = std::max(1.0e-9, unconstrainedMass(vi, vi));
        unconstrainedInertias[i] = inertia;
        nominalInertias[i] = inertia;
    }

    std::array<bool, 6> bearing{};
    bearing.fill(true);
    if (loadBearingLegs != nullptr) {
        bearing = *loadBearingLegs;
    }
    std::vector<std::size_t> bearingLegs;
    bearingLegs.reserve(6);
    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        if (bearing[leg]) {
            bearingLegs.push_back(leg);
        }
    }
    if (bearingLegs.empty()) {
        return true;
    }

    pinocchio::computeJointJacobians(model, data, q);
    Eigen::MatrixXd contactJacobian =
        Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(3U * bearingLegs.size()), nv);
    for (std::size_t rowLeg = 0; rowLeg < bearingLegs.size(); ++rowLeg) {
        const std::size_t leg = bearingLegs[rowLeg];
        const pinocchio::JointIndex tibiaJoint = wireJoints[3U * leg + 2U];
        const Eigen::Vector3d footWorld =
            ToEigen(TibiaFootWorldPosition(world.GetBody(scene.legs[leg].tibia)));
        const pinocchio::SE3 worldFoot(Eigen::Matrix3d::Identity(), footWorld);
        const pinocchio::SE3 jointToFoot = data.oMi[tibiaJoint].inverse() * worldFoot;
        Eigen::Matrix<double, 6, Eigen::Dynamic> frameJacobian =
            Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, nv);
        pinocchio::getFrameJacobian(
            model,
            data,
            tibiaJoint,
            jointToFoot,
            pinocchio::LOCAL_WORLD_ALIGNED,
            frameJacobian);
        contactJacobian.middleRows<3>(static_cast<Eigen::Index>(3U * rowLeg)) =
            frameJacobian.topRows<3>();
    }
    if (!contactJacobian.array().isFinite().all()) {
        return false;
    }

    const pinocchio::JointIndex rootJoint = model.parents[wireJoints[0]];
    const Eigen::Index vBase = model.joints[rootJoint].idx_v();
    const int nvBase = model.joints[rootJoint].nv();
    if (nvBase <= 0 || vBase < 0) {
        return false;
    }

    // Other servos are under PD, so treat them as locked and only leave the
    // free-flyer plus this joint free. Free-joint P otherwise dumps coxa
    // load into compliant femur/tibia and overstates the planted tibia.
    const int reducedSize = nvBase + 1;
    bool increasedAny = false;
    for (std::size_t i = 0; i < wireJoints.size(); ++i) {
        const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
        if (vi >= vBase && vi < vBase + nvBase) {
            continue;
        }
        Eigen::VectorXi reducedIndex(reducedSize);
        for (int k = 0; k < nvBase; ++k) {
            reducedIndex[k] = static_cast<int>(vBase + k);
        }
        reducedIndex[nvBase] = static_cast<int>(vi);

        Eigen::MatrixXd reducedMass(reducedSize, reducedSize);
        Eigen::MatrixXd reducedJacobian(contactJacobian.rows(), reducedSize);
        for (int col = 0; col < reducedSize; ++col) {
            const int fullCol = reducedIndex[col];
            reducedJacobian.col(col) = contactJacobian.col(fullCol);
            for (int row = 0; row < reducedSize; ++row) {
                reducedMass(row, col) = unconstrainedMass(reducedIndex[row], fullCol);
            }
        }

        const Eigen::LDLT<Eigen::MatrixXd> massLdlt(reducedMass);
        if (massLdlt.info() != Eigen::Success) {
            continue;
        }
        const Eigen::MatrixXd reducedMinv =
            massLdlt.solve(Eigen::MatrixXd::Identity(reducedSize, reducedSize));
        Eigen::MatrixXd delassus =
            reducedJacobian * reducedMinv * reducedJacobian.transpose();
        delassus.diagonal().array() += kStanceContactRegularization;
        const Eigen::LDLT<Eigen::MatrixXd> delassusLdlt(delassus);
        if (delassusLdlt.info() != Eigen::Success
            || !reducedMinv.array().isFinite().all()) {
            continue;
        }

        const Eigen::VectorXd freeAcceleration = reducedMinv.col(nvBase);
        const Eigen::VectorXd lambda =
            delassusLdlt.solve(reducedJacobian * freeAcceleration);
        const double inverseInertia =
            (freeAcceleration - reducedMinv * (reducedJacobian.transpose() * lambda))[nvBase];
        if (!(inverseInertia > kMinConstrainedInverseInertia)
            || !std::isfinite(inverseInertia)) {
            continue;
        }
        const double unconstrained = unconstrainedInertias[i];
        const double unclamped = 1.0 / inverseInertia;
        const double loaded = std::clamp(
            unclamped,
            unconstrained,
            maxInertiaScale * unconstrained);
        if (loaded > unconstrained) {
            increasedAny = true;
        }
        nominalInertias[i] = loaded;
        if (std::getenv("HEXAPOD_PINOCCHIO_DUMP_SERVO_INERTIAS") != nullptr
            && unclamped > kMaxStanceInertiaScale * unconstrained + 1.0e-12) {
            const char* role =
                (i % 3U == 0U) ? "coxa" : (i % 3U == 1U) ? "femur" : "tibia";
            std::cerr << "servo inertia clip L" << (i / 3U) << role
                      << " unclamped=" << unclamped
                      << " cap=" << (maxInertiaScale * unconstrained)
                      << " unconstrained=" << unconstrained << "\n";
        }
    }

    if (std::getenv("HEXAPOD_PINOCCHIO_DUMP_SERVO_INERTIAS") != nullptr) {
        std::cerr << "servo inertias unconstrained -> stance-loaded:";
        for (std::size_t i = 0; i < wireJoints.size(); ++i) {
            const char* role =
                (i % 3U == 0U) ? "coxa" : (i % 3U == 1U) ? "femur" : "tibia";
            std::cerr << " L" << (i / 3U) << role << "="
                      << unconstrainedInertias[i] << "->" << nominalInertias[i]
                      << "(x" << (nominalInertias[i] / unconstrainedInertias[i])
                      << ")";
        }
        std::cerr << "\n";
    }
    return increasedAny;
}

struct GroundContactChoice {
    const Manifold* plane = nullptr;
    const Manifold* terrain = nullptr;
    bool useTerrain = false;
};

Vec3 ExternalContactNormal(const Manifold& manifold, bool aRobot) {
    return Normalize(aRobot ? -1.0 * manifold.normal : manifold.normal);
}

bool TerrainMatchesPlane(const World& world,
                         const Manifold& terrain,
                         const Manifold& plane,
                         std::uint32_t robotBody) {
    const bool terrainARobot = terrain.a == robotBody;
    const bool planeARobot = plane.a == robotBody;
    const Vec3 terrainNormal = ExternalContactNormal(terrain, terrainARobot);
    const Vec3 planeNormal = ExternalContactNormal(plane, planeARobot);
    if (Dot(terrainNormal, planeNormal) < 0.9995) {
        return false;
    }

    const std::uint32_t planeBodyId = planeARobot ? plane.b : plane.a;
    const Body& planeBody = world.GetBody(planeBodyId);
    constexpr double kEquivalentSurfaceToleranceM = 0.003;
    for (const Contact& contact : terrain.contacts) {
        const double terrainSurfaceOffset = Dot(planeNormal, contact.point)
            + contact.penetration;
        if (std::abs(terrainSurfaceOffset - planeBody.planeOffset)
            > kEquivalentSurfaceToleranceM) {
            return false;
        }
    }
    return !terrain.contacts.empty();
}

double MotorEnvelopeAvailable(
    const double requested,
    const double velocity,
    const double stallTorque,
    const double noLoadSpeed) {
    double available = stallTorque;
    if (requested * velocity > 0.0) {
        available *= std::max(0.0, 1.0 - std::abs(velocity) / noLoadSpeed);
    }
    return available;
}

struct BoundedImplicitSolve {
    Eigen::VectorXd vFree;
    Eigen::VectorXd explicitVfree;
    Eigen::VectorXd tauChosen;
    Eigen::VectorXd tauP;
    Eigen::VectorXd damping;
    Eigen::VectorXd availableVin;
    Eigen::VectorXd availableVfree;
    Eigen::MatrixXd mass;
    Eigen::VectorXd bias;
    Eigen::MatrixXd H;
    Eigen::LDLT<Eigen::MatrixXd> Hldlt;
    bool unsaturated = true;
    bool envelopeVin = true;
    bool envelopeVfree = true;
    bool unlimitedEnvelopeHolds = true;
    bool usedUnlimitedDampingBrake = false;
    double actuatorWork = 0.0;
    double ldltRelativeError = 0.0;
    double explicitAbaRelativeError = 0.0;
};

bool SolveBoundedImplicitDamping(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const std::array<pinocchio::JointIndex, 18>& wireJoints,
    const std::array<double, 18>& servoErrors,
    const std::array<double, 18>& effectiveInertias,
    const double dt,
    const double stallTorque,
    const double noLoadSpeed,
    const double omegaN,
    const double zeta,
    const double gainScale,
    BoundedImplicitSolve& out) {
    if (!(dt > 0.0) || !std::isfinite(dt) || !IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    const Eigen::Index nv = model.nv;
    out.mass = pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
    out.mass.triangularView<Eigen::StrictlyLower>() =
        out.mass.transpose().triangularView<Eigen::StrictlyLower>();
    out.bias = pinocchio::nonLinearEffects(model, data, q, v);
    if (!out.mass.array().isFinite().all() || !IsFinite(out.bias)) {
        return false;
    }

    out.tauP = Eigen::VectorXd::Zero(nv);
    out.damping = Eigen::VectorXd::Zero(nv);
    out.availableVin = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd tauPFull = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd dampingFull = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd tauExplicit = Eigen::VectorXd::Zero(nv);
    out.unsaturated = true;
    out.envelopeVin = true;

    for (std::size_t i = 0; i < wireJoints.size(); ++i) {
        const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
        const double inertia = std::max(1.0e-12, effectiveInertias[i]);
        const double tauP = gainScale * inertia * omegaN * omegaN * servoErrors[i];
        const double damped = gainScale * 2.0 * zeta * omegaN * inertia;
        tauPFull[vi] = tauP;
        dampingFull[vi] = damped;
        const double requested = tauP - damped * v[vi];
        const double available = MotorEnvelopeAvailable(
            requested, v[vi], stallTorque, noLoadSpeed);
        out.availableVin[vi] = available;
        tauExplicit[vi] = std::clamp(requested, -available, available);
        if (std::abs(requested) > available + 1.0e-12) {
            out.unsaturated = false;
            out.tauP[vi] = tauExplicit[vi];
            out.damping[vi] = 0.0;
        } else {
            out.tauP[vi] = tauP;
            out.damping[vi] = damped;
        }
        if (std::abs(tauExplicit[vi]) > available + 1.0e-12) {
            out.envelopeVin = false;
        }
    }

    const Eigen::VectorXd explicitAcc = pinocchio::aba(
        model, data, q, v, tauExplicit, pinocchio::Convention::WORLD);
    if (!IsFinite(explicitAcc)) {
        return false;
    }
    out.explicitVfree = v + dt * explicitAcc;

    auto solveH = [&](const Eigen::VectorXd& tauP,
                      const Eigen::VectorXd& damping,
                      Eigen::VectorXd& vFree,
                      Eigen::MatrixXd& H,
                      Eigen::LDLT<Eigen::MatrixXd>& ldlt) -> bool {
        H = out.mass;
        for (Eigen::Index i = 0; i < nv; ++i) {
            H(i, i) += dt * damping[i];
        }
        ldlt.compute(H);
        if (ldlt.info() != Eigen::Success) {
            return false;
        }
        const Eigen::VectorXd rhs = out.mass * v + dt * (tauP - out.bias);
        vFree = ldlt.solve(rhs);
        return IsFinite(vFree);
    };

    if (!solveH(out.tauP, out.damping, out.vFree, out.H, out.Hldlt)) {
        return false;
    }

    for (int pass = 0; pass < 8; ++pass) {
        bool added = false;
        for (std::size_t i = 0; i < wireJoints.size(); ++i) {
            const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
            const double tau = out.tauP[vi] - out.damping[vi] * out.vFree[vi];
            const double available = MotorEnvelopeAvailable(
                tau, out.vFree[vi], stallTorque, noLoadSpeed);
            if (std::abs(tau) > available + 1.0e-12) {
                out.unsaturated = false;
                out.tauP[vi] = std::clamp(tau, -available, available);
                out.damping[vi] = 0.0;
                added = true;
            }
        }
        if (!added) {
            break;
        }
        if (!solveH(out.tauP, out.damping, out.vFree, out.H, out.Hldlt)) {
            return false;
        }
    }

    out.tauChosen = out.tauP - out.damping.cwiseProduct(out.vFree);
    out.availableVfree = Eigen::VectorXd::Zero(nv);
    out.envelopeVfree = true;
    out.actuatorWork = 0.0;
    for (std::size_t i = 0; i < wireJoints.size(); ++i) {
        const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
        const double available = MotorEnvelopeAvailable(
            out.tauChosen[vi], out.vFree[vi], stallTorque, noLoadSpeed);
        out.availableVfree[vi] = available;
        if (std::abs(out.tauChosen[vi]) > available + 1.0e-12) {
            out.envelopeVfree = false;
        }
        out.actuatorWork += out.tauChosen[vi] * out.vFree[vi] * dt;
    }

    Eigen::VectorXd vUnlimited;
    Eigen::MatrixXd Hunlimited;
    Eigen::LDLT<Eigen::MatrixXd> ldltUnlimited;
    if (!solveH(tauPFull, dampingFull, vUnlimited, Hunlimited, ldltUnlimited)) {
        return false;
    }
    out.unlimitedEnvelopeHolds = true;
    out.usedUnlimitedDampingBrake = false;
    for (std::size_t i = 0; i < wireJoints.size(); ++i) {
        const Eigen::Index vi = model.joints[wireJoints[i]].idx_v();
        const double tau = tauPFull[vi] - dampingFull[vi] * vUnlimited[vi];
        const double available = MotorEnvelopeAvailable(
            tau, vUnlimited[vi], stallTorque, noLoadSpeed);
        if (std::abs(tau) > available + 1.0e-12) {
            out.unlimitedEnvelopeHolds = false;
            if (tau * vUnlimited[vi] < 0.0) {
                out.usedUnlimitedDampingBrake = true;
            }
        }
    }

    const Eigen::VectorXd rhs = out.mass * v + dt * (out.tauP - out.bias);
    const Eigen::VectorXd residual = out.H * out.vFree - rhs;
    const Eigen::LDLT<Eigen::MatrixXd> refLdlt(out.H);
    const Eigen::VectorXd vRef = refLdlt.solve(rhs);
    const double vNorm = std::max(1.0e-16, out.vFree.norm());
    out.ldltRelativeError = std::max(
        residual.norm() / std::max(1.0e-16, rhs.norm()),
        (out.vFree - vRef).norm() / vNorm);

    if (IsFinite(out.explicitVfree)) {
        out.explicitAbaRelativeError =
            (out.vFree - out.explicitVfree).norm()
            / std::max(1.0e-16, out.explicitVfree.norm());
    }
    return IsFinite(out.vFree) && IsFinite(out.tauChosen);
}

void FillImplicitOracleResult(
    const BoundedImplicitSolve& solve,
    PinocchioHexapodModel::ImplicitDampingOracleResult& out) {
    const auto assign = [](const Eigen::VectorXd& src, std::vector<double>& dst) {
        dst.assign(src.data(), src.data() + src.size());
    };
    assign(solve.vFree, out.vFree);
    assign(solve.explicitVfree, out.explicitVfree);
    assign(solve.tauChosen, out.tauChosen);
    assign(solve.availableVin, out.availableAtVin);
    assign(solve.availableVfree, out.availableAtVfree);
    assign(solve.tauP, out.positionTorque);
    assign(solve.damping, out.dampingDiag);
    assign(solve.bias, out.bias);
    out.mass.assign(solve.mass.data(), solve.mass.data() + solve.mass.size());
    out.unsaturated = solve.unsaturated;
    out.envelopeHoldsAtVin = solve.envelopeVin;
    out.envelopeHoldsAtVfree = solve.envelopeVfree;
    out.unlimitedEnvelopeHolds = solve.unlimitedEnvelopeHolds;
    out.usedUnlimitedDampingBrake = solve.usedUnlimitedDampingBrake;
    out.actuatorWork = solve.actuatorWork;
    out.ldltRelativeError = solve.ldltRelativeError;
    out.explicitAbaRelativeError = solve.explicitAbaRelativeError;
    out.maxDelassusAbsDiff = 0.0;
}

} // namespace

using PinConstraintModels = std::vector<pinocchio::ConstraintModel>;
using PinConstraintDatas = std::vector<pinocchio::ConstraintData>;
using RigidDelassus = pinocchio::DelassusOperatorRigidBodySystemsTpl<
    double,
    0,
    pinocchio::JointCollectionDefaultTpl,
    pinocchio::ConstraintModel,
    std::reference_wrapper>;

Eigen::MatrixXd MaterializeConstraintJacobian(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const PinConstraintModels& constraintModels,
    PinConstraintDatas& constraintDatas) {
    const Eigen::Index nv = model.nv;
    const Eigen::Index nc = static_cast<Eigen::Index>(3U * constraintModels.size());
    Eigen::MatrixXd jacobian(nc, nv);
    Eigen::VectorXd basis = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd column(nc);
    for (Eigen::Index col = 0; col < nv; ++col) {
        basis.setZero();
        basis[col] = 1.0;
        pinocchio::evalConstraintJacobianMatrixProduct(
            model,
            data,
            constraintModels,
            constraintDatas,
            basis,
            column,
            pinocchio::SetTo());
        jacobian.col(col) = column;
    }
    return jacobian;
}

struct PinocchioHexapodModel::Impl {
    struct BodyBinding {
        std::uint32_t bodyId = World::kInvalidBodyId;
        pinocchio::JointIndex jointId = 0;
        pinocchio::FrameIndex frameId = 0;
    };

    pinocchio::Model model{};
    pinocchio::Data data{model};
    pinocchio::Data contactInertiaData{model};
    std::vector<BodyBinding> bodies{};
    std::array<pinocchio::JointIndex, 18> wireJoints{};
    std::array<std::uint32_t, 18> wireServoIds{};
    std::array<double, 18> wireZeroAngles{};
    std::array<double, 18> wireUnconstrainedInertias{};
    std::array<double, 18> wireNominalInertias{};
    double totalRobotMass = 0.0;
    std::unordered_map<std::uint32_t, pinocchio::JointIndex> bodyJoints{};
    std::unordered_map<std::uint32_t, std::size_t> tibiaBodyToLeg{};
    PinConstraintModels contactConstraintModels{};
    PinConstraintDatas contactConstraintDatas{};
    std::vector<std::uint64_t> contactTopologyIds{};
    std::unique_ptr<RigidDelassus> contactDelassus{};
    pinocchio::DelassusOperatorDense contactDenseDelassus{};
    double contactDelassusRegularization = 0.0;
    pinocchio::ADMMConstraintSolver contactSolver{72};
    pinocchio::ADMMSolverResult contactSolverResult{};
    std::size_t andersonCapacity = 5;
    std::optional<std::size_t> retryAndersonCapacityOverride{};
    double ratioPrimalDual = 5.0;
    double admmTau = 0.7;
    double spectralRhoPowerInit = 0.2;
    double servoGainScale = 1.0;
    std::array<double, 18> servoStiffnessNmPerRad{};
    double staticReducedSupportGainScale = kStaticReducedSupportGainScale;
    double staticReducedSupportDwellS = 0.0;
    double staticReducedSupportGainBlend = 0.0;
    bool warmstartRho = true;
    bool denseAdmm = false;
    bool contactPrecondition = false;
    bool implicitDamping = false;
    std::uint64_t contactOrderSeed = 0;

    struct WarmContact {
        Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double dt = 0.0;
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        bool haveFrame = false;
    };
    std::unordered_map<std::uint64_t, WarmContact> contactWarmStarts{};
    std::unordered_map<std::uint64_t, WarmContact> capturedWarmStarts{};

    struct AcceptedStateBufferEntry {
        std::vector<double> q{};
        std::vector<double> v{};
        std::array<double, 3> chassisAngularWorld{};
        std::array<double, 18> tau{};
        std::array<double, 18> targets{};
        std::array<double, 18> errors{};
        std::array<double, 18> effectiveInertias{};
        double servoGainScale = 1.0;
        double servoDampingGainScale = 1.0;
        double subDt = 0.0;
        double commandDt = 0.0;
        bool ncpCcpRecovery = false;
        std::size_t warmStartCount = 0;
        std::uint8_t loadBearingMask = 0;
        double reducedSupportBlend = 0.0;
        std::vector<std::uint64_t> contactIds{};
        std::vector<std::array<double, 3>> normals{};
        std::vector<std::array<double, 3>> impulses{};
    };
    static constexpr std::size_t kAcceptedStateBufferCapacity = 8;
    std::deque<AcceptedStateBufferEntry> acceptedStateBuffer{};
    bool prefailureBufferCaptured = false;
    bool standCutpointCaptured = false;
    std::uint64_t lastContactSetSignature = 0;
    bool haveLastContactSetSignature = false;
    std::vector<double> lastGoodQ{};
    std::vector<double> lastGoodV{};
    std::array<double, 18> lastServoTargets{};
    bool haveLastServoTargets = false;
    std::array<double, 18> commandedServoTargets{};
    bool haveCommandedServoTargets = false;
    bool servoTargetRecoveryActive = false;
    std::array<double, 18> wireEffectiveInertias{};
    std::array<double, 18> wireTargetInertias{};
    std::array<double, 6> lastLegNormalImpulse{};
    bool haveLegNormalImpulseHistory = false;
    std::uint8_t lastLoadBearingMask = 0;
    std::size_t lastLoadBearingCount = 6;
    bool haveLoadBearingMask = false;
    double commandIntervalS = 0.005;
    bool contactConsistentInertia = true;
    const HexapodSceneObjects* scene = nullptr;
    std::array<double, 18> staticFullSupportServoTargets{};
    bool haveStaticFullSupportServoTargets = false;
    std::array<double, 6> prevFootX{};
    std::array<double, 6> prevFootZ{};
    std::array<bool, 6> havePrevFootX{};
    std::uint64_t totalWarmStartResets = 0;
    std::uint64_t totalRetries = 0;
    std::uint64_t totalRollbacks = 0;
    std::uint64_t totalHeldStates = 0;
    std::uint64_t totalUnsupportedIslands = 0;
    // A deliberately requested failing snapshot is captured at most once per
    // model instance. This is diagnostic-only and never changes the solver.
    bool contactSnapshotCaptured = false;
    bool speedLimitSnapshotCaptured = false;

    struct CommandStreamSample {
        double dt = 0.0;
        std::array<double, 18> targets{};
        std::vector<double> q{};
        std::vector<double> v{};
    };
    static constexpr std::size_t kCommandStreamMaxSamples = 720;
    std::vector<CommandStreamSample> commandStreamSamples{};
    bool commandStreamActive = false;
    bool commandStreamWritten = false;

    void startCommandStreamIfRequested();
    void recordCommandStreamAccepted(
        double dt, const std::array<double, 18>& targets);
    void finishCommandStream(const char* reason);
    ~Impl();

    Impl(
        const World& world,
        const HexapodSceneObjects& scene,
        const std::array<std::uint32_t, 18>& servoJointIds) {
        wireServoIds = servoJointIds;
        this->scene = &scene;
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_ANDERSON_CAPACITY")) {
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (end != value && *end == '\0' && parsed >= 0 && parsed <= 16) {
                andersonCapacity = static_cast<std::size_t>(parsed);
            }
        }
        if (const char* value = std::getenv(
                "HEXAPOD_PINOCCHIO_RETRY_ANDERSON_CAPACITY")) {
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (end != value && *end == '\0' && parsed >= 0 && parsed <= 16) {
                retryAndersonCapacityOverride = static_cast<std::size_t>(parsed);
            }
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_CONTACT_ORDER_SEED")) {
            char* end = nullptr;
            const unsigned long long parsed = std::strtoull(value, &end, 10);
            if (end != value && *end == '\0') {
                contactOrderSeed = static_cast<std::uint64_t>(parsed);
            }
        }
        ratioPrimalDual = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_RATIO_PRIMAL_DUAL", ratioPrimalDual, 0.01, 1000.0);
        admmTau = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_ADMM_TAU", admmTau, 0.01, 1.0);
        spectralRhoPowerInit = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_SPECTRAL_POWER", spectralRhoPowerInit, 0.0, 1.0);
        servoGainScale = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_SERVO_GAIN_SCALE", servoGainScale, 0.01, 100.0);
        staticReducedSupportGainScale = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_STATIC_REDUCED_SUPPORT_GAIN_SCALE",
            staticReducedSupportGainScale,
            1.0,
            2.0);
        if (const char* value = std::getenv(
                "HEXAPOD_PINOCCHIO_DISABLE_CONTACT_INERTIA")) {
            contactConsistentInertia = !(value[0] != '\0' && value[0] != '0');
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_WARMSTART_RHO")) {
            warmstartRho = value[0] != '\0' && value[0] != '0';
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_DENSE_ADMM")) {
            denseAdmm = value[0] != '\0' && value[0] != '0';
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING")) {
            implicitDamping = value[0] != '\0' && value[0] != '0';
        }
        if (const char* value = std::getenv(
                "HEXAPOD_PINOCCHIO_CONTACT_PRECONDITION")) {
            contactPrecondition = value[0] != '\0' && value[0] != '0';
        }
        if (contactPrecondition) {
            if (std::getenv("HEXAPOD_PINOCCHIO_SPECTRAL_POWER") == nullptr) {
                spectralRhoPowerInit = 0.5;
            }
            if (std::getenv("HEXAPOD_PINOCCHIO_WARMSTART_RHO") == nullptr) {
                warmstartRho = false;
            }
        }

        const Body& chassis = world.GetBody(scene.body);
        const pinocchio::JointIndex root = model.addJoint(
            0,
            pinocchio::JointModelFreeFlyer(),
            pinocchio::SE3::Identity(),
            "chassis_free_flyer");
        model.appendBodyToJoint(root, BodyInertia(chassis), pinocchio::SE3::Identity());
        const pinocchio::FrameIndex chassisFrame = model.addFrame(pinocchio::Frame(
            "chassis_body", root, pinocchio::SE3::Identity(), pinocchio::BODY));
        bodies.push_back({scene.body, root, chassisFrame});
        bodyJoints.emplace(scene.body, root);

        std::unordered_map<std::uint32_t, pinocchio::JointIndex> servoToJoint;
        std::unordered_map<std::uint32_t, pinocchio::SE3> jointRestWorld;
        jointRestWorld.emplace(root, BodyPose(chassis));

        auto appendJoint = [&](std::uint32_t servoId,
                               std::uint32_t childBodyId,
                               pinocchio::JointIndex parentJoint,
                               const std::string& name) {
            const ServoJoint& servo = world.GetServoJoint(servoId);
            const Body& parentBody = world.GetBody(servo.a);
            const Body& childBody = world.GetBody(childBodyId);
            const Vec3 anchor = parentBody.position
                + Rotate(parentBody.orientation, servo.localAnchorA);
            const pinocchio::SE3 jointWorld(Eigen::Matrix3d::Identity(), ToEigen(anchor));
            const pinocchio::SE3 placement = jointRestWorld.at(parentJoint).inverse() * jointWorld;
            const Vec3 axisWorld = Normalize(Rotate(parentBody.orientation, servo.localAxisA));
            const pinocchio::JointIndex jointId = model.addJoint(
                parentJoint,
                pinocchio::JointModelRevoluteUnaligned(ToEigen(axisWorld)),
                placement,
                name);
            const pinocchio::SE3 jointToBody = jointWorld.inverse() * BodyPose(childBody);
            model.appendBodyToJoint(jointId, BodyInertia(childBody), jointToBody);
            const pinocchio::FrameIndex frameId = model.addFrame(pinocchio::Frame(
                name + "_body", jointId, jointToBody, pinocchio::BODY));
            bodies.push_back({childBodyId, jointId, frameId});
            bodyJoints.emplace(childBodyId, jointId);
            servoToJoint.emplace(servoId, jointId);
            jointRestWorld.emplace(jointId, jointWorld);
            return jointId;
        };

        for (std::size_t legIndex = 0; legIndex < scene.legs.size(); ++legIndex) {
            const LegLinkIds& leg = scene.legs[legIndex];
            const std::string prefix = "leg_" + std::to_string(legIndex);
            const pinocchio::JointIndex coxa = appendJoint(
                leg.bodyToCoxaJoint, leg.coxa, root, prefix + "_coxa");
            const pinocchio::JointIndex femur = appendJoint(
                leg.coxaToFemurJoint, leg.femur, coxa, prefix + "_femur");
            (void)appendJoint(
                leg.femurToTibiaJoint, leg.tibia, femur, prefix + "_tibia");
            tibiaBodyToLeg.emplace(leg.tibia, legIndex);
        }

        for (const BodyBinding& binding : bodies) {
            totalRobotMass += world.GetBody(binding.bodyId).mass;
        }

        for (std::size_t i = 0; i < wireServoIds.size(); ++i) {
            wireJoints[i] = servoToJoint.at(wireServoIds[i]);
            wireZeroAngles[i] = world.GetServoJointAngle(wireServoIds[i]);
        }

        model.gravity.linear() = Eigen::Vector3d(0.0, -9.80665, 0.0);
        data = pinocchio::Data(model);
        contactInertiaData = pinocchio::Data(model);
    }
};

PinocchioHexapodModel::PinocchioHexapodModel(
    const World& world,
    const HexapodSceneObjects& scene,
    const std::array<std::uint32_t, 18>& servo_joint_ids)
    : impl_(std::make_unique<Impl>(world, scene, servo_joint_ids)) {
    if (!readState(world, impl_->lastGoodQ, impl_->lastGoodV)) {
        throw std::runtime_error("failed to capture initial Pinocchio hexapod state");
    }
    ProximalStepDiagnostics initialDiagnostics{};
    for (const auto& binding : impl_->bodies) {
        if (!BodyStateWithinBounds(world.GetBody(binding.bodyId), {})) {
            throw std::runtime_error("initial hexapod body violates dynamic bounds");
        }
    }
    if (!validateDynamicState(impl_->lastGoodQ, impl_->lastGoodV, {}, initialDiagnostics)) {
        throw std::runtime_error("initial Pinocchio hexapod state violates dynamic bounds");
    }
    const Eigen::Map<const Eigen::VectorXd> initialQ(
        impl_->lastGoodQ.data(), static_cast<Eigen::Index>(impl_->lastGoodQ.size()));
    Eigen::MatrixXd nominalMass = pinocchio::crba(
        impl_->model, impl_->data, initialQ, pinocchio::Convention::WORLD);
    nominalMass.triangularView<Eigen::StrictlyLower>() =
        nominalMass.transpose().triangularView<Eigen::StrictlyLower>();
    if (!nominalMass.array().isFinite().all()) {
        throw std::runtime_error("failed to compute nominal hexapod joint inertias");
    }
    if (!AssignStanceLoadedServoInertias(
            impl_->model,
            impl_->data,
            world,
            scene,
            initialQ,
            impl_->wireJoints,
            nominalMass,
            impl_->wireUnconstrainedInertias,
            impl_->wireNominalInertias)) {
        for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
            const Eigen::Index vi = impl_->model.joints[impl_->wireJoints[i]].idx_v();
            const double inertia = std::max(1.0e-9, nominalMass(vi, vi));
            impl_->wireUnconstrainedInertias[i] = inertia;
            impl_->wireNominalInertias[i] = inertia;
        }
    }
    impl_->wireEffectiveInertias = impl_->wireNominalInertias;
    impl_->wireTargetInertias = impl_->wireNominalInertias;
    if (impl_->contactConsistentInertia) {
        for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
            const Eigen::Index vi = impl_->model.joints[impl_->wireJoints[i]].idx_v();
            impl_->model.armature[vi] =
                kServoArmatureInertiaScale * impl_->wireUnconstrainedInertias[i];
        }
    }
}

PinocchioHexapodModel::~PinocchioHexapodModel() = default;
PinocchioHexapodModel::PinocchioHexapodModel(PinocchioHexapodModel&&) noexcept = default;
PinocchioHexapodModel& PinocchioHexapodModel::operator=(PinocchioHexapodModel&&) noexcept = default;

std::size_t PinocchioHexapodModel::configurationSize() const {
    return static_cast<std::size_t>(impl_->model.nq);
}

std::size_t PinocchioHexapodModel::velocitySize() const {
    return static_cast<std::size_t>(impl_->model.nv);
}

std::size_t PinocchioHexapodModel::jointCount() const {
    return impl_->wireJoints.size();
}

std::array<double, 18> PinocchioHexapodModel::servoUnconstrainedInertias() const {
    return impl_->wireUnconstrainedInertias;
}

std::array<double, 18> PinocchioHexapodModel::servoNominalInertias() const {
    return impl_->wireNominalInertias;
}

void PinocchioHexapodModel::setCommandInterval(double seconds) {
    if (std::isfinite(seconds) && seconds > 0.0) {
        impl_->commandIntervalS = seconds;
    }
}

bool PinocchioHexapodModel::readState(
    const World& world,
    std::vector<double>& qOut,
    std::vector<double>& vOut) const {
    Eigen::VectorXd q = pinocchio::neutral(impl_->model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv);
    const Body& chassis = world.GetBody(impl_->bodies.front().bodyId);
    q.segment<3>(0) = ToEigen(chassis.position);
    q[3] = chassis.orientation.x;
    q[4] = chassis.orientation.y;
    q[5] = chassis.orientation.z;
    q[6] = chassis.orientation.w;
    const Eigen::Matrix3d worldToBody = ToEigenRotation(chassis.orientation).transpose();
    v.segment<3>(0) = worldToBody * ToEigen(chassis.velocity);
    v.segment<3>(3) = worldToBody * ToEigen(chassis.angularVelocity);

    for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
        const pinocchio::JointIndex pinJoint = impl_->wireJoints[i];
        const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
        const Body& parent = world.GetBody(servo.a);
        const Body& child = world.GetBody(servo.b);
        q[impl_->model.joints[pinJoint].idx_q()] = std::remainder(
            world.GetServoJointAngle(impl_->wireServoIds[i]) - impl_->wireZeroAngles[i],
            6.28318530717958647692);
        const Vec3 axisWorld = Normalize(Rotate(parent.orientation, servo.localAxisA));
        v[impl_->model.joints[pinJoint].idx_v()] =
            Dot(child.angularVelocity - parent.angularVelocity, axisWorld);
    }

    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    qOut.assign(q.data(), q.data() + q.size());
    vOut.assign(v.data(), v.data() + v.size());
    return true;
}

bool PinocchioHexapodModel::writeState(
    World& world,
    const std::vector<double>& qIn,
    const std::vector<double>& vIn) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), static_cast<Eigen::Index>(qIn.size()));
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), static_cast<Eigen::Index>(vIn.size()));
    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        Body& body = world.GetBody(binding.bodyId);
        const pinocchio::SE3& pose = impl_->data.oMf[binding.frameId];
        const pinocchio::Motion motion = pinocchio::getFrameVelocity(
            impl_->model,
            impl_->data,
            binding.frameId,
            pinocchio::LOCAL_WORLD_ALIGNED);
        body.position = FromEigen(pose.translation());
        body.orientation = FromEigenRotation(pose.rotation());
        body.velocity = FromEigen(motion.linear());
        body.angularVelocity = FromEigen(motion.angular());
        body.isSleeping = false;
    }
    return true;
}

bool PinocchioHexapodModel::computeFreeAcceleration(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    const std::vector<double>& tauIn,
    std::vector<double>& ddqOut) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()
        || tauIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    const Eigen::Map<const Eigen::VectorXd> tau(tauIn.data(), impl_->model.nv);
    const Eigen::VectorXd ddq = pinocchio::aba(impl_->model, impl_->data, q, v, tau);
    if (!IsFinite(ddq)) {
        return false;
    }
    ddqOut.assign(ddq.data(), ddq.data() + ddq.size());
    return true;
}

bool PinocchioHexapodModel::computeDenseAcceleration(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    const std::vector<double>& tauIn,
    std::vector<double>& ddqOut) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()
        || tauIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    const Eigen::Map<const Eigen::VectorXd> tau(tauIn.data(), impl_->model.nv);
    Eigen::MatrixXd mass = pinocchio::crba(impl_->model, impl_->data, q);
    mass.triangularView<Eigen::StrictlyLower>() = mass.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::VectorXd bias = pinocchio::nonLinearEffects(impl_->model, impl_->data, q, v);
    const Eigen::VectorXd ddq = mass.ldlt().solve(tau - bias);
    if (!IsFinite(ddq)) {
        return false;
    }
    ddqOut.assign(ddq.data(), ddq.data() + ddq.size());
    return true;
}

bool PinocchioHexapodModel::validateDelassusOracle(
    const World& world,
    double tolerance,
    double& maxRelativeError) {
    maxRelativeError = 0.0;
    if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
        return false;
    }

    std::vector<double> qStorage;
    std::vector<double> vStorage;
    if (!readState(world, qStorage, vStorage)) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qStorage.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vStorage.data(), impl_->model.nv);
    pinocchio::crba(impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    const pinocchio::JointIndex chassis = impl_->bodies.front().jointId;
    const Body& chassisBody = world.GetBody(impl_->bodies.front().bodyId);
    const Eigen::Matrix3d rotation = ContactFrameRotation({0.0, 1.0, 0.0});
    constexpr double regularization = 1.0e-10;
    const auto makeContact = [&](pinocchio::JointIndex joint,
                                 const Eigen::Vector3d& worldPoint) {
        const pinocchio::SE3 worldContact(rotation, worldPoint);
        const pinocchio::SE3 jointPlacement =
            impl_->data.oMi[joint].inverse() * worldContact;
        return pinocchio::ConstraintModel(pinocchio::PointContactConstraintModel(
            impl_->model, 0, worldContact, joint, jointPlacement));
    };
    const auto compare = [&](PinConstraintModels models) {
        PinConstraintDatas datas;
        datas.reserve(models.size());
        for (const pinocchio::ConstraintModel& model : models) {
            datas.push_back(model.createData());
        }
        for (std::size_t i = 0; i < models.size(); ++i) {
            models[i].calc(impl_->model, impl_->data, datas[i]);
        }

        RigidDelassus rigid(
            std::cref(impl_->model),
            std::ref(impl_->data),
            std::cref(models),
            std::cref(datas),
            regularization);
        rigid.compute();
        pinocchio::ConstraintCholeskyDecomposition cholesky(
            impl_->model, impl_->data, models, datas, regularization);
        cholesky.compute(impl_->model, impl_->data, models, datas, regularization);
        const Eigen::MatrixXd production = rigid.matrix(false, true);
        const Eigen::MatrixXd oracle =
            cholesky.getDelassusOperatorCholeskyExpression().matrix(false, true);
        if (production.rows() != oracle.rows() || production.cols() != oracle.cols()
            || !production.array().isFinite().all() || !oracle.array().isFinite().all()) {
            return false;
        }
        const double scale = std::max(1.0, oracle.cwiseAbs().maxCoeff());
        const double relativeError =
            (production - oracle).cwiseAbs().maxCoeff() / scale;
        maxRelativeError = std::max(maxRelativeError, relativeError);
        return std::isfinite(relativeError) && relativeError <= tolerance;
    };

    const Eigen::Vector3d chassisPoint =
        ToEigen(chassisBody.position) + Eigen::Vector3d(0.0, -0.08, 0.11);
    const pinocchio::ConstraintModel chassisContact = makeContact(chassis, chassisPoint);
    if (!compare({chassisContact})
        || !compare({
            chassisContact,
            makeContact(
                chassis,
                ToEigen(chassisBody.position) + Eigen::Vector3d(0.12, -0.13, -0.07))})
        || !compare({chassisContact, chassisContact})) {
        return false;
    }

    PinConstraintModels allFeet;
    allFeet.reserve(6);
    for (std::size_t leg = 0; leg < 6; ++leg) {
        const pinocchio::JointIndex tibiaJoint = impl_->wireJoints[3U * leg + 2U];
        allFeet.push_back(makeContact(tibiaJoint, impl_->data.oMi[tibiaJoint].translation()));
    }
    return compare(std::move(allFeet));
}

void PinocchioHexapodModel::resetWarmStarts() {
    impl_->totalWarmStartResets += impl_->contactWarmStarts.size();
    impl_->contactWarmStarts.clear();
    impl_->lastContactSetSignature = 0;
    impl_->haveLastContactSetSignature = false;
    impl_->contactSolverResult.reset();
    impl_->contactSolver.reset();
}

void PinocchioHexapodModel::maybePollCutpointFileIpc(World& world) {
    std::error_code existsError;
    if (const char* dumpReq = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_REQUEST");
        dumpReq != nullptr && dumpReq[0] != '\0'
        && std::filesystem::exists(dumpReq, existsError)) {
        const char* dumpPath = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_PATH");
        if (dumpPath != nullptr && dumpPath[0] != '\0') {
            if (std::filesystem::exists(dumpPath, existsError)) {
                std::cerr << "[proximal-cutpoint-dump] skip existing path=" << dumpPath << '\n';
            } else if (dumpDiagnosticCutpoint(dumpPath)) {
                std::cerr << "[proximal-cutpoint-dump] path=" << dumpPath
                          << " warm_starts=" << impl_->contactWarmStarts.size() << '\n';
            } else {
                std::cerr << "[proximal-cutpoint-dump] failed path=" << dumpPath << '\n';
            }
        }
        std::filesystem::remove(dumpReq, existsError);
    }
    if (const char* restoreReq = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_REQUEST");
        restoreReq != nullptr && restoreReq[0] != '\0'
        && std::filesystem::exists(restoreReq, existsError)) {
        const char* restorePath = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_PATH");
        const char* jsonPath =
            (restorePath != nullptr && restorePath[0] != '\0') ? restorePath : restoreReq;
        const bool unlinkTrigger = restorePath != nullptr && restorePath[0] != '\0';
        if (restoreDiagnosticCutpoint(world, jsonPath)) {
            const char* clearWarms = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_CLEAR_WARMS");
            const bool clear = clearWarms != nullptr && clearWarms[0] == '1' && clearWarms[1] == '\0';
            if (clear) {
                resetWarmStarts();
            }
            std::cerr << "[proximal-cutpoint-restore] path=" << jsonPath
                      << " clear_warms=" << (clear ? 1 : 0)
                      << " warm_starts=" << impl_->contactWarmStarts.size() << '\n';
        } else {
            std::cerr << "[proximal-cutpoint-restore] failed path=" << jsonPath << '\n';
        }
        if (unlinkTrigger) {
            std::filesystem::remove(restoreReq, existsError);
        }
    }
}

bool PinocchioHexapodModel::debugLastServoBalance(ServoBalanceSample& out) const {
    out = {};
    if (impl_->acceptedStateBuffer.empty()) return false;
    const auto& sample = impl_->acceptedStateBuffer.back();
    out.errors = sample.errors;
    out.appliedTorques = sample.tau;
    out.effectiveInertias = sample.effectiveInertias;
    out.gainScale = sample.servoGainScale;
    out.dampingGainScale = sample.servoDampingGainScale;
    out.subDt = sample.subDt;
    return true;
}

std::array<double, 18> PinocchioHexapodModel::servoStiffnessNmPerRad() const {
    return impl_->servoStiffnessNmPerRad;
}

bool PinocchioHexapodModel::computeMechanicalEnergyBreakdown(
    const World& world,
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    MechanicalEnergyBreakdown& out) const {
    out = {};
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    Eigen::MatrixXd mass = pinocchio::crba(
        impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
    mass.triangularView<Eigen::StrictlyLower>() =
        mass.transpose().triangularView<Eigen::StrictlyLower>();
    if (!mass.array().isFinite().all()) {
        return false;
    }
    out.generalizedKinetic = 0.5 * v.dot(mass * v);
    for (int i = 0; i < impl_->model.nv; ++i) {
        out.armatureKinetic += 0.5 * impl_->model.armature[i] * v[i] * v[i];
    }
    const Vec3 gravity = world.GetGravity();
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        const Body& body = world.GetBody(binding.bodyId);
        Mat3 inertiaWorld{};
        const Mat3 inverseWorld = body.InvInertiaWorld();
        if (!InvertMat3(inverseWorld, inertiaWorld)) {
            return false;
        }
        out.bodyKinetic += 0.5 * body.mass * Dot(body.velocity, body.velocity);
        out.bodyKinetic += 0.5 * Dot(body.angularVelocity, inertiaWorld * body.angularVelocity);
        out.potential -= body.mass * Dot(gravity, body.position);
    }
    return std::isfinite(out.generalizedKinetic) && std::isfinite(out.bodyKinetic)
        && std::isfinite(out.armatureKinetic) && std::isfinite(out.potential);
}

bool PinocchioHexapodModel::computeLinkWorldTwist(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    std::uint32_t bodyId,
    Vec3& linearWorld,
    Vec3& angularWorld) const {
    linearWorld = {};
    angularWorld = {};
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        if (binding.bodyId != bodyId) {
            continue;
        }
        const auto motion = pinocchio::getFrameVelocity(
            impl_->model, impl_->data, binding.frameId, pinocchio::LOCAL_WORLD_ALIGNED);
        linearWorld = {motion.linear().x(), motion.linear().y(), motion.linear().z()};
        angularWorld = {motion.angular().x(), motion.angular().y(), motion.angular().z()};
        return std::isfinite(Length(linearWorld)) && std::isfinite(Length(angularWorld));
    }
    return false;
}

bool PinocchioHexapodModel::computeLinkAngularJacobian(
    const std::vector<double>& qIn,
    std::uint32_t bodyId,
    std::vector<double>& columnMajor3xNv) const {
    columnMajor3xNv.clear();
    if (qIn.size() != configurationSize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    if (!IsFinite(q)) {
        return false;
    }
    pinocchio::computeJointJacobians(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        if (binding.bodyId != bodyId) {
            continue;
        }
        Eigen::Matrix<double, 6, Eigen::Dynamic> frameJacobian(6, impl_->model.nv);
        frameJacobian.setZero();
        pinocchio::getFrameJacobian(
            impl_->model,
            impl_->data,
            binding.frameId,
            pinocchio::LOCAL_WORLD_ALIGNED,
            frameJacobian);
        const Eigen::MatrixXd angular = frameJacobian.bottomRows<3>();
        if (!angular.array().isFinite().all()) {
            return false;
        }
        columnMajor3xNv.assign(angular.data(), angular.data() + angular.size());
        return true;
    }
    return false;
}

bool PinocchioHexapodModel::computeImplicitDampingOracle(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    const std::array<double, 18>& servoErrors,
    const std::array<double, 18>& effectiveInertias,
    const double dt,
    ImplicitDampingOracleResult& out,
    const double gainScale,
    const std::vector<double>* constraintJacobian,
    const std::size_t jacobianRows) const {
    out = {};
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    const double stallTorque =
        hexapod_dynamics::kServoMaxTorqueNm * servoTorqueScaleFromEnv();
    constexpr double noLoadSpeed = hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
    constexpr double omegaN = hexapod_dynamics::kServoOmegaN;
    constexpr double zeta = hexapod_dynamics::kServoZeta;
    BoundedImplicitSolve solve;
    if (!SolveBoundedImplicitDamping(
            impl_->model,
            impl_->data,
            q,
            v,
            impl_->wireJoints,
            servoErrors,
            effectiveInertias,
            dt,
            stallTorque,
            noLoadSpeed,
            omegaN,
            zeta,
            gainScale,
            solve)) {
        return false;
    }
    FillImplicitOracleResult(solve, out);
    if (constraintJacobian != nullptr && jacobianRows > 0) {
        const Eigen::Index nv = impl_->model.nv;
        const Eigen::Index rows = static_cast<Eigen::Index>(jacobianRows);
        if (constraintJacobian->size() != static_cast<std::size_t>(rows * nv)) {
            return false;
        }
        const Eigen::Map<const Eigen::MatrixXd> jacobian(
            constraintJacobian->data(), rows, nv);
        if (!jacobian.array().isFinite().all()) {
            return false;
        }
        const Eigen::LDLT<Eigen::MatrixXd> massLdlt(solve.mass);
        if (massLdlt.info() != Eigen::Success || solve.Hldlt.info() != Eigen::Success) {
            return false;
        }
        const Eigen::MatrixXd gM = jacobian * massLdlt.solve(jacobian.transpose());
        const Eigen::MatrixXd gH = jacobian * solve.Hldlt.solve(jacobian.transpose());
        out.maxDelassusAbsDiff = (gH - gM).cwiseAbs().maxCoeff();
    }
    return true;
}

PinocchioHexapodModel::WarmStartAudit PinocchioHexapodModel::debugWarmStartAudit() const {
    WarmStartAudit audit{};
    audit.count = impl_->contactWarmStarts.size();
    for (const auto& entry : impl_->contactWarmStarts) {
        audit.impulseNorm += entry.second.impulse.squaredNorm();
        if (entry.second.haveFrame) {
            ++audit.framedCount;
        }
    }
    audit.impulseNorm = std::sqrt(audit.impulseNorm);
    return audit;
}

void PinocchioHexapodModel::debugCaptureWarmStarts() {
    impl_->capturedWarmStarts = impl_->contactWarmStarts;
}

void PinocchioHexapodModel::debugRestoreCapturedWarmStarts() {
    impl_->contactWarmStarts = impl_->capturedWarmStarts;
}

void PinocchioHexapodModel::debugRotateWarmStartTangentBasis(double radians) {
    if (!std::isfinite(radians) || radians == 0.0) {
        return;
    }
    for (auto& entry : impl_->contactWarmStarts) {
        Impl::WarmContact& warm = entry.second;
        if (!warm.haveFrame) {
            continue;
        }
        const Eigen::Vector3d normal = warm.frame.col(2).normalized();
        const Eigen::Matrix3d rotated =
            Eigen::AngleAxisd(radians, normal).toRotationMatrix() * warm.frame;
        warm.impulse = TransportContactFrameVector(warm.frame, rotated, warm.impulse);
        warm.velocity = TransportContactFrameVector(warm.frame, rotated, warm.velocity);
        warm.frame = rotated;
    }
}

bool PinocchioHexapodModel::dumpDiagnosticCutpoint(const char* path) const {
    if (path == nullptr || path[0] == '\0') {
        return false;
    }
    std::ofstream fixture(path, std::ios::trunc);
    if (!fixture) {
        return false;
    }
    fixture << std::setprecision(17);
    fixture << "{\"schema_version\":1,\"kind\":\"stand_cutpoint\"";
    fixture << ",\"warm_start_count\":" << impl_->contactWarmStarts.size();
    fixture << ",\"load_bearing_mask\":" << static_cast<unsigned>(impl_->lastLoadBearingMask);
    fixture << ",\"load_bearing_count\":" << impl_->lastLoadBearingCount;
    fixture << ",\"reduced_support_dwell_s\":" << impl_->staticReducedSupportDwellS;
    fixture << ",\"reduced_support_blend\":" << impl_->staticReducedSupportGainBlend;
    fixture << ",\"command_interval_s\":" << impl_->commandIntervalS;
    fixture << ",\"q\":";
    WriteJsonNumberArray(fixture, impl_->lastGoodQ);
    fixture << ",\"v\":";
    WriteJsonNumberArray(fixture, impl_->lastGoodV);
    fixture << ",\"last_servo_targets\":";
    WriteJsonNumberArray(fixture, impl_->lastServoTargets);
    fixture << ",\"effective_inertias\":";
    WriteJsonNumberArray(fixture, impl_->wireEffectiveInertias);
    fixture << ",\"target_inertias\":";
    WriteJsonNumberArray(fixture, impl_->wireTargetInertias);
    fixture << ",\"nominal_inertias\":";
    WriteJsonNumberArray(fixture, impl_->wireNominalInertias);
    fixture << ",\"last_leg_normal_impulse\":";
    WriteJsonNumberArray(fixture, impl_->lastLegNormalImpulse);
    fixture << ",\"warm_starts\":[";
    bool first = true;
    for (const auto& entry : impl_->contactWarmStarts) {
        if (!first) {
            fixture << ',';
        }
        first = false;
        fixture << "{\"id\":" << entry.first
                << ",\"dt\":" << entry.second.dt
                << ",\"impulse\":[" << entry.second.impulse.x() << ','
                << entry.second.impulse.y() << ',' << entry.second.impulse.z()
                << "],\"velocity\":[" << entry.second.velocity.x() << ','
                << entry.second.velocity.y() << ',' << entry.second.velocity.z()
                << "],\"have_frame\":" << (entry.second.haveFrame ? "true" : "false");
        if (entry.second.haveFrame) {
            const Eigen::Matrix3d& frame = entry.second.frame;
            fixture << ",\"frame\":[["
                    << frame(0, 0) << ',' << frame(0, 1) << ',' << frame(0, 2)
                    << "],["
                    << frame(1, 0) << ',' << frame(1, 1) << ',' << frame(1, 2)
                    << "],["
                    << frame(2, 0) << ',' << frame(2, 1) << ',' << frame(2, 2)
                    << "]]";
        }
        fixture << "}";
    }
    fixture << "]}\n";
    return static_cast<bool>(fixture);
}

bool PinocchioHexapodModel::applyServoTargets(
    World& world, const std::array<double, 18>& targets) {
    for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
        world.GetServoJointMutable(impl_->wireServoIds[i]).targetAngle = targets[i];
    }
    return true;
}

bool PinocchioHexapodModel::restoreAcceptedHistorySample(
    World& world, const DiagnosticHistorySample& sample) {
    if (sample.q.size() != configurationSize() || sample.v.size() != velocitySize()) {
        return false;
    }
    impl_->lastGoodQ = sample.q;
    impl_->lastGoodV = sample.v;
    if (!writeState(world, sample.q, sample.v)) {
        return false;
    }
    impl_->lastServoTargets = sample.targets;
    impl_->commandedServoTargets = sample.targets;
    impl_->haveLastServoTargets = true;
    impl_->haveCommandedServoTargets = true;
    impl_->servoTargetRecoveryActive = false;
    impl_->wireEffectiveInertias = sample.effectiveInertias;
    impl_->staticReducedSupportGainBlend = sample.reducedSupportBlend;
    if (sample.commandDt > 0.0 && std::isfinite(sample.commandDt)) {
        impl_->commandIntervalS = sample.commandDt;
    }
    impl_->lastLoadBearingMask = sample.loadBearingMask;
    std::size_t bearing = 0;
    for (unsigned bit = 0; bit < 6; ++bit) {
        if ((sample.loadBearingMask & static_cast<std::uint8_t>(1u << bit)) != 0) {
            ++bearing;
        }
    }
    impl_->lastLoadBearingCount = bearing;
    impl_->haveLoadBearingMask = true;

    std::unordered_map<std::uint64_t, Impl::WarmContact> warmStarts;
    for (const DiagnosticHistoryContact& contact : sample.contacts) {
        Impl::WarmContact warm;
        warm.dt = sample.subDt;
        warm.impulse = {contact.impulse[0], contact.impulse[1], contact.impulse[2]};
        warm.velocity = Eigen::Vector3d::Zero();
        const Vec3 normal{contact.normal[0], contact.normal[1], contact.normal[2]};
        const double n2 = Dot(normal, normal);
        if (std::isfinite(contact.normal[0]) && std::isfinite(contact.normal[1])
            && std::isfinite(contact.normal[2]) && n2 > 1.0e-24) {
            warm.frame = ContactFrameRotation(normal);
            warm.haveFrame = true;
        } else {
            warm.frame = Eigen::Matrix3d::Identity();
            warm.haveFrame = false;
        }
        warmStarts[contact.id] = warm;
    }
    impl_->contactWarmStarts = std::move(warmStarts);
    if (!applyServoTargets(world, sample.targets)) {
        return false;
    }
    const double contactDt = sample.subDt > 0.0 ? sample.subDt
        : (impl_->commandIntervalS > 0.0 ? impl_->commandIntervalS : 0.005);
    world.PrepareExternalContacts(contactDt);
    world.CompleteExternalDynamicsStep();
    return true;
}

void PinocchioHexapodModel::Impl::startCommandStreamIfRequested() {
    if (commandStreamActive || commandStreamWritten) {
        return;
    }
    const char* path = std::getenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    std::error_code existsError;
    if (std::filesystem::exists(path, existsError)) {
        commandStreamWritten = true;
        std::cerr << "[proximal-command-stream] skip existing path=" << path << '\n';
        return;
    }
    commandStreamActive = true;
    commandStreamSamples.clear();
    commandStreamSamples.reserve(kCommandStreamMaxSamples);
}

void PinocchioHexapodModel::Impl::recordCommandStreamAccepted(
    double dt, const std::array<double, 18>& targets) {
    if (!commandStreamActive || commandStreamWritten) {
        return;
    }
    CommandStreamSample sample;
    sample.dt = dt;
    sample.targets = targets;
    sample.q = lastGoodQ;
    sample.v = lastGoodV;
    commandStreamSamples.push_back(std::move(sample));
    if (commandStreamSamples.size() >= kCommandStreamMaxSamples) {
        finishCommandStream("max_samples");
    }
}

void PinocchioHexapodModel::Impl::finishCommandStream(const char* reason) {
    if (!commandStreamActive || commandStreamWritten) {
        commandStreamActive = false;
        return;
    }
    commandStreamActive = false;
    const char* path = std::getenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
    if (path == nullptr || path[0] == '\0' || commandStreamSamples.empty()) {
        commandStreamWritten = true;
        return;
    }
    std::error_code existsError;
    if (std::filesystem::exists(path, existsError)) {
        commandStreamWritten = true;
        std::cerr << "[proximal-command-stream] skip existing path=" << path << '\n';
        return;
    }
    std::ofstream fixture(path, std::ios::out);
    if (!fixture) {
        std::cerr << "[proximal-command-stream] failed path=" << path << '\n';
        commandStreamWritten = true;
        return;
    }
    fixture << std::setprecision(17);
    fixture << "{\"schema_version\":1,\"kind\":\"command_stream\"";
    fixture << ",\"implicit_damping\":" << (implicitDamping ? "true" : "false");
    fixture << ",\"steps\":[";
    for (std::size_t i = 0; i < commandStreamSamples.size(); ++i) {
        if (i != 0) {
            fixture << ',';
        }
        const CommandStreamSample& sample = commandStreamSamples[i];
        fixture << "{\"dt\":" << sample.dt << ",\"targets\":";
        WriteJsonNumberArray(fixture, sample.targets);
        fixture << ",\"q\":";
        WriteJsonNumberArray(fixture, sample.q);
        fixture << ",\"v\":";
        WriteJsonNumberArray(fixture, sample.v);
        fixture << "}";
    }
    fixture << "]}\n";
    commandStreamWritten = true;
    std::cerr << "[proximal-command-stream] path=" << path
              << " samples=" << commandStreamSamples.size()
              << " reason=" << (reason != nullptr ? reason : "unknown") << '\n';
}

PinocchioHexapodModel::Impl::~Impl() {
    finishCommandStream("shutdown");
}

bool PinocchioHexapodModel::restoreDiagnosticCutpoint(World& world, const char* path) {
    std::string text;
    if (!LoadEntireFile(path, text)) {
        return false;
    }
    const std::size_t nposTo = text.size();
    std::string kind;
    if (!ExtractJsonQuoted(text, "kind", 0, nposTo, kind) || kind != "stand_cutpoint") {
        return false;
    }
    double schema = 0.0;
    std::size_t schemaPos = 0;
    if (!FindJsonKeyInRange(text, "schema_version", 0, nposTo, schemaPos)
        || !ParseJsonNumber(text, schemaPos, schema)
        || schema != 1.0) {
        return false;
    }

    auto extractArray = [&](const char* key, std::vector<double>& out) {
        std::size_t valuePos = 0;
        if (!FindJsonKeyInRange(text, key, 0, nposTo, valuePos)) {
            return false;
        }
        return ParseJsonNumberArray(text, valuePos, out);
    };
    auto extractNumber = [&](const char* key, double& out) {
        std::size_t valuePos = 0;
        if (!FindJsonKeyInRange(text, key, 0, nposTo, valuePos)) {
            return false;
        }
        return ParseJsonNumber(text, valuePos, out);
    };

    std::vector<double> q;
    std::vector<double> v;
    std::vector<double> lastTargets;
    std::vector<double> effective;
    std::vector<double> targetInertias;
    std::vector<double> nominal;
    std::vector<double> legImpulse;
    if (!extractArray("q", q) || !extractArray("v", v)
        || !extractArray("last_servo_targets", lastTargets)
        || !extractArray("effective_inertias", effective)
        || !extractArray("target_inertias", targetInertias)
        || !extractArray("nominal_inertias", nominal)
        || !extractArray("last_leg_normal_impulse", legImpulse)) {
        return false;
    }
    if (q.size() != configurationSize() || v.size() != velocitySize()
        || lastTargets.size() != 18 || effective.size() != 18
        || targetInertias.size() != 18 || nominal.size() != 18
        || legImpulse.size() != 6) {
        return false;
    }

    double maskValue = 0.0;
    double countValue = 6.0;
    double dwell = 0.0;
    double blend = 0.0;
    double commandInterval = impl_->commandIntervalS;
    if (!extractNumber("load_bearing_mask", maskValue)
        || !extractNumber("load_bearing_count", countValue)
        || !extractNumber("reduced_support_dwell_s", dwell)
        || !extractNumber("reduced_support_blend", blend)) {
        return false;
    }
    (void)extractNumber("command_interval_s", commandInterval);

    std::unordered_map<std::uint64_t, Impl::WarmContact> warmStarts;
    std::size_t warmPos = 0;
    if (FindJsonKeyInRange(text, "warm_starts", 0, nposTo, warmPos)) {
        SkipJsonWhitespace(text, warmPos);
        if (warmPos >= text.size() || text[warmPos] != '[') {
            return false;
        }
        ++warmPos;
        SkipJsonWhitespace(text, warmPos);
        while (warmPos < text.size() && text[warmPos] != ']') {
            SkipJsonWhitespace(text, warmPos);
            if (warmPos >= text.size() || text[warmPos] != '{') {
                return false;
            }
            const std::size_t objectStart = warmPos;
            int depth = 0;
            std::size_t objectEnd = warmPos;
            for (; objectEnd < text.size(); ++objectEnd) {
                if (text[objectEnd] == '{') {
                    ++depth;
                } else if (text[objectEnd] == '}') {
                    --depth;
                    if (depth == 0) {
                        ++objectEnd;
                        break;
                    }
                }
            }
            if (depth != 0) {
                return false;
            }
            double idValue = 0.0;
            double dtValue = 0.0;
            std::size_t fieldPos = 0;
            if (!FindJsonKeyInRange(text, "id", objectStart, objectEnd, fieldPos)
                || !ParseJsonNumber(text, fieldPos, idValue)
                || !FindJsonKeyInRange(text, "dt", objectStart, objectEnd, fieldPos)
                || !ParseJsonNumber(text, fieldPos, dtValue)) {
                return false;
            }
            std::vector<double> impulse;
            std::vector<double> velocity;
            if (!FindJsonKeyInRange(text, "impulse", objectStart, objectEnd, fieldPos)
                || !ParseJsonNumberArray(text, fieldPos, impulse)
                || impulse.size() != 3
                || !FindJsonKeyInRange(text, "velocity", objectStart, objectEnd, fieldPos)
                || !ParseJsonNumberArray(text, fieldPos, velocity)
                || velocity.size() != 3) {
                return false;
            }
            Impl::WarmContact warm;
            warm.dt = dtValue;
            warm.impulse = {impulse[0], impulse[1], impulse[2]};
            warm.velocity = {velocity[0], velocity[1], velocity[2]};
            bool haveFrame = false;
            if (FindJsonKeyInRange(text, "have_frame", objectStart, objectEnd, fieldPos)) {
                (void)ParseJsonBool(text, fieldPos, haveFrame);
            }
            std::vector<double> frameValues;
            if (FindJsonKeyInRange(text, "frame", objectStart, objectEnd, fieldPos)
                && ParseJsonNumberArray(text, fieldPos, frameValues)
                && frameValues.size() == 9) {
                warm.frame << frameValues[0], frameValues[1], frameValues[2],
                    frameValues[3], frameValues[4], frameValues[5],
                    frameValues[6], frameValues[7], frameValues[8];
                warm.haveFrame = haveFrame;
            } else {
                warm.haveFrame = false;
                warm.frame = Eigen::Matrix3d::Identity();
            }
            warmStarts[static_cast<std::uint64_t>(idValue)] = warm;
            warmPos = objectEnd;
            SkipJsonWhitespace(text, warmPos);
            if (warmPos < text.size() && text[warmPos] == ',') {
                ++warmPos;
            }
        }
    }

    impl_->lastGoodQ = q;
    impl_->lastGoodV = v;
    if (!writeState(world, q, v)) {
        return false;
    }
    if (!CopyJsonArray(lastTargets, impl_->lastServoTargets)
        || !CopyJsonArray(lastTargets, impl_->commandedServoTargets)
        || !CopyJsonArray(lastTargets, impl_->staticFullSupportServoTargets)
        || !CopyJsonArray(effective, impl_->wireEffectiveInertias)
        || !CopyJsonArray(targetInertias, impl_->wireTargetInertias)
        || !CopyJsonArray(nominal, impl_->wireNominalInertias)
        || !CopyJsonArray(legImpulse, impl_->lastLegNormalImpulse)) {
        return false;
    }
    impl_->haveLastServoTargets = true;
    impl_->haveCommandedServoTargets = true;
    impl_->haveStaticFullSupportServoTargets = true;
    impl_->servoTargetRecoveryActive = false;
    impl_->staticReducedSupportDwellS = dwell;
    impl_->staticReducedSupportGainBlend = blend;
    impl_->commandIntervalS = commandInterval;
    impl_->lastLoadBearingMask = static_cast<std::uint8_t>(maskValue);
    impl_->lastLoadBearingCount = static_cast<std::size_t>(countValue);
    impl_->haveLoadBearingMask = true;
    impl_->haveLegNormalImpulseHistory = true;
    impl_->contactWarmStarts = std::move(warmStarts);
    impl_->standCutpointCaptured = true;
    if (!applyServoTargets(world, impl_->lastServoTargets)) {
        return false;
    }
    const double contactDt = impl_->commandIntervalS > 0.0 ? impl_->commandIntervalS : 0.005;
    world.PrepareExternalContacts(contactDt);
    world.CompleteExternalDynamicsStep();
    return true;
}

bool PinocchioHexapodModel::validateDynamicState(
    const std::vector<double>& qIn, const std::vector<double>& vIn,
    const ProximalSolverSettings& settings, ProximalStepDiagnostics& diagnostics) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        diagnostics.failureReason = ProximalFailureReason::NonFiniteState;
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), static_cast<Eigen::Index>(qIn.size()));
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), static_cast<Eigen::Index>(vIn.size()));
    if (!IsFinite(q) || !IsFinite(v) || std::abs(q.segment<4>(3).norm() - 1.0) > 1e-6) {
        diagnostics.failureReason = ProximalFailureReason::NonFiniteState;
        return false;
    }
    if (!std::isfinite(settings.maxLinearSpeed) || settings.maxLinearSpeed <= 0.0
        || !std::isfinite(settings.maxAngularSpeed) || settings.maxAngularSpeed <= 0.0) {
        diagnostics.failureReason = ProximalFailureReason::SpeedLimit;
        return false;
    }
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);
    const Impl::BodyBinding* winner = nullptr;
    double winnerSeverity = -1.0;
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        const auto motion = pinocchio::getFrameVelocity(
            impl_->model, impl_->data, binding.frameId, pinocchio::LOCAL_WORLD_ALIGNED);
        const double linear = motion.linear().norm(), angular = motion.angular().norm();
        diagnostics.preIntegrationLinearSpeed = std::max(diagnostics.preIntegrationLinearSpeed, linear);
        diagnostics.preIntegrationAngularSpeed = std::max(diagnostics.preIntegrationAngularSpeed, angular);
        if (binding.bodyId == impl_->bodies.front().bodyId) {
            diagnostics.chassisPreIntegrationAngularSpeed = std::max(diagnostics.chassisPreIntegrationAngularSpeed, angular);
        } else {
            diagnostics.maxLinkPreIntegrationAngularSpeed = std::max(diagnostics.maxLinkPreIntegrationAngularSpeed, angular);
        }
        if (std::isfinite(linear) && std::isfinite(angular)
            && linear <= settings.maxLinearSpeed && angular <= settings.maxAngularSpeed) continue;
        const double severity = std::isfinite(linear) && std::isfinite(angular)
            ? std::max(linear / settings.maxLinearSpeed, angular / settings.maxAngularSpeed)
            : std::numeric_limits<double>::infinity();
        if (!winner || severity >= winnerSeverity) { winner = &binding; winnerSeverity = severity; }
    }
    if (winner) {
        const auto& name = impl_->model.frames[winner->frameId].name;
        diagnostics.speedLimitFrame = SpeedLimitFrameFromName(name);
        if (const auto leg = SpeedLimitLegIndexFromName(name); leg.has_value()) {
            diagnostics.speedLimitLegIndex = static_cast<int>(*leg);
        }
        diagnostics.failureReason = ProximalFailureReason::SpeedLimit;
        return false;
    }
    return true;
}

bool PinocchioHexapodModel::writeValidatedState(
    World& world, const std::vector<double>& q, const std::vector<double>& v,
    const ProximalSolverSettings& settings, ProximalStepDiagnostics& diagnostics) {
    if (!validateDynamicState(q, v, settings, diagnostics)) return false;
    if (!writeState(world, q, v)) {
        diagnostics.failureReason = ProximalFailureReason::WriteState;
        return false;
    }
    return true;
}

bool PinocchioHexapodModel::synchronizeAfterExternalCorrection(
    World& world, const ProximalSolverSettings& settings) {
    std::vector<double> correctedQ;
    std::vector<double> correctedV;
    ProximalStepDiagnostics correctionDiagnostics{};
    bool rawSafe = true;
    for (const auto& binding : impl_->bodies) {
        rawSafe = rawSafe && BodyStateWithinBounds(world.GetBody(binding.bodyId), settings);
    }
    if (!rawSafe || !readState(world, correctedQ, correctedV)
        || !validateDynamicState(correctedQ, correctedV, settings, correctionDiagnostics)) {
        (void)writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        return false;
    }

    resetWarmStarts();
    impl_->lastGoodQ = std::move(correctedQ);
    impl_->lastGoodV = std::move(correctedV);
    impl_->haveLastServoTargets = false;
    impl_->haveCommandedServoTargets = false;
    impl_->servoTargetRecoveryActive = false;
    impl_->haveStaticFullSupportServoTargets = false;
    impl_->staticReducedSupportDwellS = 0.0;
    impl_->staticReducedSupportGainBlend = 0.0;
    impl_->wireEffectiveInertias = impl_->wireNominalInertias;
    impl_->wireTargetInertias = impl_->wireNominalInertias;
    impl_->lastLegNormalImpulse.fill(0.0);
    impl_->haveLegNormalImpulseHistory = false;
    impl_->lastLoadBearingMask = 0;
    impl_->lastLoadBearingCount = 6;
    impl_->haveLoadBearingMask = false;
    return true;
}

bool PinocchioHexapodModel::stepProximal(
    World& world,
    double dt,
    const ProximalSolverSettings& settings,
    ProximalStepDiagnostics& diagnostics) {
    using StepClock = std::chrono::steady_clock;
    const auto stepStart = StepClock::now();
    const auto elapsedMs = [](const StepClock::time_point start) {
        return std::chrono::duration<double, std::milli>(StepClock::now() - start).count();
    };
    diagnostics = {};
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        diagnostics.failureReason = ProximalFailureReason::InvalidDt;
        diagnostics.totalStepTimeMs = elapsedMs(stepStart);
        impl_->finishCommandStream("invalid_dt");
        return false;
    }
    maybePollCutpointFileIpc(world);

    std::array<double, 18> servoTargets{};
    bool targetJump = !impl_->haveLastServoTargets;
    double maxServoTargetDelta = 0.0;
    for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
        servoTargets[i] = world.GetServoJoint(impl_->wireServoIds[i]).targetAngle;
        if (impl_->haveLastServoTargets) {
            const double delta = std::abs(std::remainder(
                servoTargets[i] - impl_->lastServoTargets[i],
                6.28318530717958647692));
            maxServoTargetDelta = std::max(maxServoTargetDelta, delta);
            targetJump = targetJump || delta > 0.25;
        }
    }
    const bool resetCommandedTargets = targetJump && impl_->haveLastServoTargets;
    if (!impl_->standCutpointCaptured && impl_->haveLastServoTargets
        && impl_->haveStaticFullSupportServoTargets
        && !impl_->contactWarmStarts.empty()
        && maxServoTargetDelta > 1.0e-6) {
        bool marked = false;
        if (const char* cutPath = std::getenv("HEXAPOD_PINOCCHIO_STAND_CUTPOINT_PATH");
            cutPath != nullptr && cutPath[0] != '\0') {
            if (dumpDiagnosticCutpoint(cutPath)) {
                impl_->standCutpointCaptured = true;
                marked = true;
                std::cerr << "[proximal-stand-cutpoint] path=" << cutPath
                          << " warm_starts=" << impl_->contactWarmStarts.size()
                          << " target_delta=" << maxServoTargetDelta << '\n';
            }
        } else {
            impl_->standCutpointCaptured = true;
            marked = true;
        }
        if (marked) {
            impl_->startCommandStreamIfRequested();
        }
    }
    impl_->lastServoTargets = servoTargets;
    impl_->haveLastServoTargets = true;
    // A discontinuous servo command does not invalidate foot-contact impulses;
    // wiping them on stand-to-walk target jumps starts a held-state cascade.
    (void)resetCommandedTargets;

    std::vector<double> snapshotQ;
    std::vector<double> snapshotV;
    if (!readState(world, snapshotQ, snapshotV)) {
        ++impl_->totalHeldStates;
        ++impl_->totalRollbacks;
        (void)writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        impl_->servoTargetRecoveryActive = true;
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        diagnostics.failureReason = ProximalFailureReason::ReadState;
        diagnostics.warmStartResets = impl_->totalWarmStartResets;
        diagnostics.retries = impl_->totalRetries;
        diagnostics.rollbackCount = impl_->totalRollbacks;
        diagnostics.heldStateCount = impl_->totalHeldStates;
        diagnostics.unsupportedIslandCount = impl_->totalUnsupportedIslands;
        diagnostics.totalStepTimeMs = elapsedMs(stepStart);
        impl_->finishCommandStream("read_state");
        return false;
    }
    if (impl_->lastGoodQ.empty()) {
        impl_->lastGoodQ = snapshotQ;
        impl_->lastGoodV = snapshotV;
    }
    // The server already applies the configured command slew limit, so healthy
    // steps use its target directly. A held step is different: the dynamic
    // state rolls back while the external command stream keeps advancing. If
    // the distant target is reapplied immediately, the same SpeedLimit failure
    // repeats forever. Rejoin the stream from the last-good joint angles at a
    // loaded-feasible rate, then return to the unfiltered healthy path.
    const std::array<double, 18> previousCommanded = impl_->haveCommandedServoTargets
        ? impl_->commandedServoTargets
        : servoTargets;
    const bool recoverySlewActive = impl_->servoTargetRecoveryActive
        && impl_->haveCommandedServoTargets;
    std::array<double, 18> commandedServoTargets = servoTargets;
    if (recoverySlewActive) {
        constexpr double kRecoveryTargetRateRadps =
            0.5 * hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
        const double maxDelta = kRecoveryTargetRateRadps * dt;
        bool reachedAllTargets = true;
        for (std::size_t i = 0; i < commandedServoTargets.size(); ++i) {
            const double delta = std::remainder(
                servoTargets[i] - impl_->commandedServoTargets[i],
                6.28318530717958647692);
            if (std::abs(delta) > maxDelta) {
                commandedServoTargets[i] =
                    impl_->commandedServoTargets[i] + std::copysign(maxDelta, delta);
                reachedAllTargets = false;
            }
        }
        impl_->servoTargetRecoveryActive = !reachedAllTargets;
    }

    auto totalMechanicalEnergy = [&]() {
        const Vec3 gravity = world.GetGravity();
        double energy = 0.0;
        for (const Impl::BodyBinding& binding : impl_->bodies) {
            const Body& body = world.GetBody(binding.bodyId);
            Mat3 inertiaWorld{};
            const Mat3 inverseWorld = body.InvInertiaWorld();
            if (!InvertMat3(inverseWorld, inertiaWorld)) {
                continue;
            }
            energy += 0.5 * body.mass * Dot(body.velocity, body.velocity);
            energy += 0.5 * Dot(body.angularVelocity, inertiaWorld * body.angularVelocity);
            energy -= body.mass * Dot(gravity, body.position);
        }
        std::vector<double> energyQ;
        std::vector<double> energyV;
        if (readState(world, energyQ, energyV)
            && energyV.size() == static_cast<std::size_t>(impl_->model.nv)) {
            for (int i = 0; i < impl_->model.nv; ++i) {
                energy += 0.5 * impl_->model.armature[i] * energyV[static_cast<std::size_t>(i)]
                    * energyV[static_cast<std::size_t>(i)];
            }
        }
        return energy;
    };

    bool unsupportedIsland = false;
    bool contactInertiaRetargetedThisStep = false;
    const bool traceSpeedLimit = []() {
        const char* trace = std::getenv("HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT");
        return trace != nullptr && trace[0] != '\0' && trace[0] != '0';
    }();
    const bool transportWarmStarts = []() {
        const char* value = std::getenv("HEXAPOD_PINOCCHIO_TRANSPORT_WARM_START");
        return value && std::string(value) == "1";
    }();
    // Keep the explicit motor's damping when reducing retry position demand.
    // Healthy steps remain identical;
    // do not silently change the separate implicit actuator/contact operator.
    const bool retryKeepDamping = !impl_->implicitDamping && []() {
        const char* value = std::getenv("HEXAPOD_PINOCCHIO_RETRY_KEEP_DAMPING");
        return value == nullptr || std::string(value) != "0";
    }();
    const bool traceFailures = []() {
        const char* trace = std::getenv("HEXAPOD_PROXIMAL_TRACE_FAILURES");
        return trace != nullptr && trace[0] != '\0' && trace[0] != '0';
    }();
    const bool disableNcpCcpRecovery = []() {
        const char* value = std::getenv("HEXAPOD_PINOCCHIO_DISABLE_NCP_CCP_RECOVERY");
        return value != nullptr && value[0] != '\0' && value[0] != '0';
    }();
    const bool compliantContactSession = settings.compliantContact || []() {
        const char* value = std::getenv("HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT");
        return value != nullptr && value[0] != '\0' && value[0] != '0';
    }();
    auto writeAcceptedHistoryJson = [&](std::ostream& out) {
        out << "\"accepted_history\":[";
        bool firstSample = true;
        for (const auto& sample : impl_->acceptedStateBuffer) {
            if (!firstSample) {
                out << ',';
            }
            firstSample = false;
            out << "{\"sub_dt\":" << sample.subDt
                << ",\"servo_gain_scale\":" << sample.servoGainScale
                << ",\"servo_damping_gain_scale\":" << sample.servoDampingGainScale
                << ",\"command_dt\":" << sample.commandDt
                << ",\"ncp_ccp_recovery\":" << (sample.ncpCcpRecovery ? "true" : "false")
                << ",\"warm_start_count\":" << sample.warmStartCount
                << ",\"load_bearing_mask\":" << static_cast<unsigned>(sample.loadBearingMask)
                << ",\"reduced_support_blend\":" << sample.reducedSupportBlend
                << ",\"chassis_angular_world\":";
            WriteJsonNumberArray(out, sample.chassisAngularWorld);
            out << ",\"q\":";
            WriteJsonNumberArray(out, sample.q);
            out << ",\"v\":";
            WriteJsonNumberArray(out, sample.v);
            out << ",\"tau\":";
            WriteJsonNumberArray(out, sample.tau);
            out << ",\"targets\":";
            WriteJsonNumberArray(out, sample.targets);
            out << ",\"errors\":";
            WriteJsonNumberArray(out, sample.errors);
            out << ",\"effective_inertias\":";
            WriteJsonNumberArray(out, sample.effectiveInertias);
            out << ",\"contacts\":[";
            for (std::size_t i = 0; i < sample.contactIds.size(); ++i) {
                if (i != 0) {
                    out << ',';
                }
                out << "{\"id\":" << sample.contactIds[i] << ",\"normal\":[";
                if (i < sample.normals.size()) {
                    out << sample.normals[i][0] << ',' << sample.normals[i][1] << ','
                        << sample.normals[i][2];
                } else {
                    out << "0,0,0";
                }
                out << "],\"impulse\":[";
                if (i < sample.impulses.size()) {
                    out << sample.impulses[i][0] << ',' << sample.impulses[i][1] << ','
                        << sample.impulses[i][2];
                } else {
                    out << "0,0,0";
                }
                out << "]}";
            }
            out << "]}";
        }
        out << ']';
    };
    auto dumpPrefailureBuffer = [&](const char* trigger) {
        if (impl_->prefailureBufferCaptured) {
            return;
        }
        const char* path = std::getenv("HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH");
        if (path == nullptr || path[0] == '\0') {
            return;
        }
        try {
            std::error_code existsError;
            if (std::filesystem::exists(path, existsError)) {
                impl_->prefailureBufferCaptured = true;
                std::cerr << "[proximal-prefailure-buffer] skip existing path=" << path << '\n';
                return;
            }
            std::ofstream fixture(path, std::ios::trunc);
            if (!fixture) {
                throw std::runtime_error("cannot open prefailure buffer path");
            }
            fixture << std::setprecision(17);
            fixture << "{\"schema_version\":1,\"kind\":\"prefailure_buffer\",\"trigger\":"
                    << JsonEscape(trigger) << ',';
            writeAcceptedHistoryJson(fixture);
            fixture << "}\n";
            impl_->prefailureBufferCaptured = true;
            std::cerr << "[proximal-prefailure-buffer] path=" << path
                      << " trigger=" << trigger
                      << " samples=" << impl_->acceptedStateBuffer.size() << '\n';
        } catch (const std::exception& ex) {
            std::cerr << "[proximal-prefailure-buffer] failed: " << ex.what() << '\n';
        }
    };
    bool loggedImplicitThisStep = false;
    auto advanceOnce = [&](double subDt,
                           const std::array<double, 18>& activeServoTargets,
                           const std::size_t andersonCapacity,
                           ProximalStepDiagnostics& out,
                           const int iterationOverride = 0,
                           const double pdGainScale = 1.0,
                           const double ncpAcceptAbsoluteOverride = 0.0,
                           const std::uint64_t omitContactId = 0,
                           const double minPenetration = 0.0,
                           const bool forceCompliantContact = false) -> bool {
        const auto dynamicsStart = StepClock::now();
        std::vector<double> qStorage;
        std::vector<double> vStorage;
        if (!readState(world, qStorage, vStorage)) {
            out.failureReason = ProximalFailureReason::ReadState;
            return false;
        }
        Eigen::Map<const Eigen::VectorXd> q(qStorage.data(), impl_->model.nq);
        Eigen::Map<const Eigen::VectorXd> v(vStorage.data(), impl_->model.nv);
        if (!IsFinite(q) || !IsFinite(v)) {
            out.failureReason = ProximalFailureReason::NonFiniteState;
            return false;
        }

        impl_->model.gravity.linear() = ToEigen(world.GetGravity());
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(impl_->model.nv);
        const double stallTorque =
            hexapod_dynamics::kServoMaxTorqueNm * servoTorqueScaleFromEnv();
        constexpr double noLoadSpeed = hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
        constexpr double omegaN = hexapod_dynamics::kServoOmegaN;
        constexpr double zeta = hexapod_dynamics::kServoZeta;
        std::array<bool, 6> supportingLegs{};
        for (const Manifold& manifold : world.DebugManifolds()) {
            if (manifold.contacts.empty()) {
                continue;
            }
            const bool aRobot = impl_->bodyJoints.find(manifold.a) != impl_->bodyJoints.end();
            const bool bRobot = impl_->bodyJoints.find(manifold.b) != impl_->bodyJoints.end();
            if (aRobot == bRobot) {
                continue;
            }
            const std::uint32_t robotBody = aRobot ? manifold.a : manifold.b;
            const auto legIt = impl_->tibiaBodyToLeg.find(robotBody);
            if (legIt != impl_->tibiaBodyToLeg.end()) {
                supportingLegs[legIt->second] = true;
            }
        }
        const std::size_t supportLegCount = static_cast<std::size_t>(
            std::count(supportingLegs.begin(), supportingLegs.end(), true));
        bool stationaryTargets = impl_->haveCommandedServoTargets;
        for (std::size_t i = 0; stationaryTargets && i < activeServoTargets.size(); ++i) {
            stationaryTargets = std::abs(std::remainder(
                activeServoTargets[i] - impl_->commandedServoTargets[i],
                6.28318530717958647692)) < 1.0e-5;
        }
        std::array<bool, 6> loadBearingLegs{};
        const double loadBearingFloor = std::max(
            1.0e-6,
            kLoadBearingNormalImpulseFraction * impl_->totalRobotMass * 9.80665 * subDt);
        std::size_t loadBearingCount = 0;
        for (std::size_t leg = 0; leg < 6; ++leg) {
            if (!supportingLegs[leg]) {
                continue;
            }
            const bool bearing = !impl_->haveLegNormalImpulseHistory
                || impl_->lastLegNormalImpulse[leg] >= loadBearingFloor;
            loadBearingLegs[leg] = bearing;
            if (bearing) {
                ++loadBearingCount;
            }
        }
        std::uint8_t loadBearingMask = 0;
        for (std::size_t leg = 0; leg < 6; ++leg) {
            if (loadBearingLegs[leg]) {
                loadBearingMask |= static_cast<std::uint8_t>(1U << leg);
            }
        }
        const bool productionCommandPeriod =
            impl_->commandIntervalS >= 0.0045 && impl_->commandIntervalS <= 0.0055;
        const bool reducedContactInertia =
            impl_->contactConsistentInertia && productionCommandPeriod;
        const bool maskChanged = !impl_->haveLoadBearingMask
            || loadBearingMask != impl_->lastLoadBearingMask;
        if (maskChanged) {
            if (!reducedContactInertia || loadBearingCount == 0U || loadBearingCount == 6U) {
                impl_->wireTargetInertias = (loadBearingCount == 0U && reducedContactInertia)
                    ? impl_->wireUnconstrainedInertias
                    : impl_->wireNominalInertias;
                impl_->lastLoadBearingMask = loadBearingMask;
                impl_->lastLoadBearingCount = loadBearingCount;
                impl_->haveLoadBearingMask = true;
            } else if (!contactInertiaRetargetedThisStep && impl_->scene != nullptr) {
                contactInertiaRetargetedThisStep = true;
                Eigen::MatrixXd currentMass = pinocchio::crba(
                    impl_->model, impl_->contactInertiaData, q, pinocchio::Convention::WORLD);
                currentMass.triangularView<Eigen::StrictlyLower>() =
                    currentMass.transpose().triangularView<Eigen::StrictlyLower>();
                std::array<double, 18> unconstrainedScratch{};
                std::array<double, 18> loadedScratch{};
                if (currentMass.array().isFinite().all()
                    && AssignStanceLoadedServoInertias(
                        impl_->model,
                        impl_->contactInertiaData,
                        world,
                        *impl_->scene,
                        q,
                        impl_->wireJoints,
                        currentMass,
                        unconstrainedScratch,
                        loadedScratch,
                        &loadBearingLegs,
                        kMaxReducedContactInertiaScale)) {
                    impl_->wireTargetInertias = loadedScratch;
                    impl_->lastLoadBearingMask = loadBearingMask;
                    impl_->lastLoadBearingCount = loadBearingCount;
                    impl_->haveLoadBearingMask = true;
                    if (std::getenv("HEXAPOD_PINOCCHIO_DUMP_SERVO_INERTIAS") != nullptr) {
                        std::cerr << "contact-crba mask=" << static_cast<unsigned>(loadBearingMask)
                                  << " count=" << loadBearingCount << "\n";
                    }
                }
            }
        }
        const double inertiaBlendAlpha = std::min(1.0, subDt / kContactInertiaRampS);
        for (std::size_t i = 0; i < impl_->wireEffectiveInertias.size(); ++i) {
            impl_->wireEffectiveInertias[i] += inertiaBlendAlpha
                * (impl_->wireTargetInertias[i] - impl_->wireEffectiveInertias[i]);
        }
        // Infer the requested support pattern from commands because the wire
        // protocol intentionally carries joint targets, not gait/contact intent.
        // Latch the first stable six-foot command and never replace it during
        // the subsequent unload transition.
        if (!impl_->haveStaticFullSupportServoTargets
            && stationaryTargets && supportLegCount == 6U) {
            impl_->staticFullSupportServoTargets = activeServoTargets;
            impl_->haveStaticFullSupportServoTargets = true;
        }
        std::size_t changedLegCount = 0;
        if (impl_->haveStaticFullSupportServoTargets) {
            for (std::size_t leg = 0; leg < 6; ++leg) {
                double maxLegTargetDelta = 0.0;
                for (std::size_t joint = 0; joint < 3; ++joint) {
                    const std::size_t wire = 3U * leg + joint;
                    maxLegTargetDelta = std::max(
                        maxLegTargetDelta,
                        std::abs(std::remainder(
                            activeServoTargets[wire]
                                - impl_->staticFullSupportServoTargets[wire],
                            6.28318530717958647692)));
                }
                if (maxLegTargetDelta > kStaticReducedSupportLegTargetDeltaRad) {
                    ++changedLegCount;
                }
            }
        }
        const bool staticReducedSupportCandidate =
            stationaryTargets
            && impl_->haveStaticFullSupportServoTargets
            && changedLegCount == 3U
            && supportLegCount > 0U
            && supportLegCount < 6U;
        if (staticReducedSupportCandidate) {
            impl_->staticReducedSupportDwellS += subDt;
            if (impl_->staticReducedSupportDwellS >= kStaticReducedSupportDwellS) {
                impl_->staticReducedSupportGainBlend = std::min(
                    1.0,
                    impl_->staticReducedSupportGainBlend
                        + subDt / kStaticReducedSupportRampS);
            }
        } else {
            impl_->staticReducedSupportDwellS = 0.0;
            impl_->staticReducedSupportGainBlend = 0.0;
        }
        const double reducedSupportGain = 1.0
            + (impl_->staticReducedSupportGainScale - 1.0)
                * impl_->staticReducedSupportGainBlend;
        const bool reducedContactCrba = loadBearingCount > 0U && loadBearingCount < 6U;
        const double loadGainScale =
            (staticReducedSupportCandidate && !reducedContactCrba)
            ? reducedSupportGain
            : 1.0;
        double peakPdAbsError = 0.0;
        std::array<double, 18> requestedTau{};
        std::array<double, 18> availableTau{};
        std::array<double, 18> servoErrors{};
        for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
            const pinocchio::JointIndex joint = impl_->wireJoints[i];
            const Eigen::Index qi = impl_->model.joints[joint].idx_q();
            const Eigen::Index vi = impl_->model.joints[joint].idx_v();
            const double error = std::remainder(
                activeServoTargets[i] - impl_->wireZeroAngles[i] - q[qi],
                6.28318530717958647692);
            servoErrors[i] = error;
            peakPdAbsError = std::max(peakPdAbsError, std::abs(error));
            const double reflectedInertia = impl_->wireEffectiveInertias[i];
            impl_->servoStiffnessNmPerRad[i] = impl_->servoGainScale * loadGainScale
                * reflectedInertia * omegaN * omegaN;
            // Reduced-contact CRBA is the load model. Do not stack the static
            // 1.85× latch on top of it.
            const double requested = retryKeepDamping && pdGainScale != 1.0
                ? impl_->servoGainScale * loadGainScale * servoPdRequest(
                    reflectedInertia * omegaN * omegaN,
                    2.0 * zeta * omegaN * reflectedInertia,
                    error, v[vi], pdGainScale, 1.0)
                : impl_->servoGainScale * loadGainScale * pdGainScale
                    * (reflectedInertia * omegaN * omegaN * error
                        - 2.0 * zeta * omegaN * reflectedInertia * v[vi]);
            double available = stallTorque;
            if (requested * v[vi] > 0.0) {
                available *= std::max(0.0, 1.0 - std::abs(v[vi]) / noLoadSpeed);
            }
            requestedTau[i] = requested;
            availableTau[i] = available;
            tau[vi] = std::clamp(requested, -available, available);
            out.peakActuatorImpulse = std::max(out.peakActuatorImpulse, std::abs(tau[vi]) * subDt);
            out.peakServoTorqueUtilization = std::max(
                out.peakServoTorqueUtilization, std::abs(tau[vi]) / stallTorque);
            out.actuatorWork += tau[vi] * v[vi] * subDt;
        }
        out.peakPdAbsError = peakPdAbsError;

        const double energyBefore = totalMechanicalEnergy();
        BoundedImplicitSolve implicitSolve;
        bool haveImplicitH = false;
        Eigen::MatrixXd implicitGUndamped;
        Eigen::VectorXd vNew;
        if (impl_->implicitDamping) {
            const double implicitGain = impl_->servoGainScale * loadGainScale * pdGainScale;
            if (!SolveBoundedImplicitDamping(
                    impl_->model,
                    impl_->data,
                    q,
                    v,
                    impl_->wireJoints,
                    servoErrors,
                    impl_->wireEffectiveInertias,
                    subDt,
                    stallTorque,
                    noLoadSpeed,
                    omegaN,
                    zeta,
                    implicitGain,
                    implicitSolve)) {
                out.failureReason = ProximalFailureReason::NonFiniteAcceleration;
                return false;
            }
            vNew = implicitSolve.vFree;
            tau = implicitSolve.tauChosen;
            haveImplicitH = true;
            out.actuatorWork = implicitSolve.actuatorWork;
            out.peakActuatorImpulse = 0.0;
            out.peakServoTorqueUtilization = 0.0;
            for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
                const Eigen::Index vi = impl_->model.joints[impl_->wireJoints[i]].idx_v();
                out.peakActuatorImpulse = std::max(
                    out.peakActuatorImpulse, std::abs(tau[vi]) * subDt);
                out.peakServoTorqueUtilization = std::max(
                    out.peakServoTorqueUtilization, std::abs(tau[vi]) / stallTorque);
            }
            pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
            pinocchio::updateFramePlacements(impl_->model, impl_->data);
            if (!loggedImplicitThisStep) {
                loggedImplicitThisStep = true;
                std::cerr << "[proximal-implicit-damping]"
                          << " dt=" << subDt
                          << " unsaturated=" << (implicitSolve.unsaturated ? 1 : 0)
                          << " envelope_vin=" << (implicitSolve.envelopeVin ? 1 : 0)
                          << " envelope_vfree=" << (implicitSolve.envelopeVfree ? 1 : 0)
                          << " ldlt_rel=" << implicitSolve.ldltRelativeError
                          << '\n';
            }
        } else {
            const Eigen::VectorXd acceleration = pinocchio::aba(
                impl_->model, impl_->data, q, v, tau, pinocchio::Convention::WORLD);
            vNew = v + subDt * acceleration;
            if (!IsFinite(acceleration) || !IsFinite(vNew)) {
                out.failureReason = ProximalFailureReason::NonFiniteAcceleration;
                return false;
            }
        }
        if (!IsFinite(vNew)) {
            out.failureReason = ProximalFailureReason::NonFiniteAcceleration;
            return false;
        }
        out.dynamicsTimeMs += elapsedMs(dynamicsStart);
        const Eigen::VectorXd vBeforeContact = vNew;

        const auto contactSetupStart = StepClock::now();
        world.PrepareExternalContacts(subDt);
        out.collisionTimeMs += elapsedMs(contactSetupStart);
        auto constraintAssemblyStart = StepClock::now();
        std::vector<std::uint64_t> contactIds;
        std::vector<double> contactFrictions;
        std::vector<double> contactRestitutions;
        std::vector<double> contactPenetrations;
        std::unordered_set<std::uint64_t> seenContactIds;
        std::unordered_map<std::uint32_t, GroundContactChoice> groundChoices;
        for (const Manifold& candidate : world.DebugManifolds()) {
            const bool aRobot = impl_->bodyJoints.find(candidate.a) != impl_->bodyJoints.end();
            const bool bRobot = impl_->bodyJoints.find(candidate.b) != impl_->bodyJoints.end();
            if (aRobot == bRobot) {
                continue;
            }
            const std::uint32_t robotBody = aRobot ? candidate.a : candidate.b;
            const std::uint32_t externalBody = aRobot ? candidate.b : candidate.a;
            if (world.IsTerrainAttachmentBody(externalBody)) {
                groundChoices[robotBody].terrain = &candidate;
            } else if (world.GetBody(externalBody).shape == ShapeType::Plane) {
                groundChoices[robotBody].plane = &candidate;
            }
        }
        for (auto& [robotBody, choice] : groundChoices) {
            choice.useTerrain = choice.terrain != nullptr
                && (choice.plane == nullptr
                    || !TerrainMatchesPlane(world, *choice.terrain, *choice.plane, robotBody));
        }

        struct AcceptedContactGeometry {
            pinocchio::JointIndex joint1 = 0;
            pinocchio::JointIndex joint2 = 0;
            Vec3 point{};
            Vec3 normal{};
        };
        struct PendingContact {
            AcceptedContactGeometry geometry{};
            std::uint64_t id = 0;
            double friction = 0.0;
            double restitution = 0.0;
            double penetration = 0.0;
            std::size_t legIndex = 6;
            Vec3 pointVelocity{};
            Vec3 tibiaVelocity{};
            Vec3 spinVelocity{};
            Vec3 footVelocity{};
        };
        std::vector<AcceptedContactGeometry> acceptedContactGeometry;
        std::vector<PendingContact> pendingContacts;
        std::vector<std::array<double, 3>> bufferNormals;
        std::vector<std::array<double, 3>> bufferImpulses;
        contactIds.reserve(world.DebugManifolds().size() * 4U);
        pendingContacts.reserve(world.DebugManifolds().size() * 4U);

        for (const Manifold& manifold : world.DebugManifolds()) {
            const auto aIt = impl_->bodyJoints.find(manifold.a);
            const auto bIt = impl_->bodyJoints.find(manifold.b);
            const bool aRobot = aIt != impl_->bodyJoints.end();
            const bool bRobot = bIt != impl_->bodyJoints.end();
            if (!aRobot && !bRobot) {
                continue;
            }
            if (aRobot != bRobot) {
                const std::uint32_t robotBody = aRobot ? manifold.a : manifold.b;
                const std::uint32_t externalBody = aRobot ? manifold.b : manifold.a;
                const auto choiceIt = groundChoices.find(robotBody);
                if (choiceIt != groundChoices.end()) {
                    const bool isTerrain = world.IsTerrainAttachmentBody(externalBody);
                    const bool isPlane = !isTerrain
                        && world.GetBody(externalBody).shape == ShapeType::Plane;
                    if ((isTerrain && !choiceIt->second.useTerrain)
                        || (isPlane && choiceIt->second.useTerrain)) {
                        continue;
                    }
                }
            }
            if (aRobot && bRobot) {
                const ServoJoint* connectingJoint = nullptr;
                for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
                    const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
                    if ((servo.a == manifold.a && servo.b == manifold.b)
                        || (servo.a == manifold.b && servo.b == manifold.a)) {
                        connectingJoint = &servo;
                        break;
                    }
                }
                if (connectingJoint != nullptr) {
                    continue;
                }
            }
            ++out.contactManifoldCount;
            if (aRobot && bRobot) {
                ++out.robotRobotManifoldCount;
            } else {
                ++out.externalManifoldCount;
            }
            if (aRobot != bRobot) {
                const std::uint32_t externalId = aRobot ? manifold.b : manifold.a;
                const Body& external = world.GetBody(externalId);
                if (!external.isStatic && external.invMass > 0.0) {
                    unsupportedIsland = true;
                    out.failureReason = ProximalFailureReason::UnsupportedIsland;
                    return false;
                }
            }

            const Body& bodyA = world.GetBody(manifold.a);
            const Body& bodyB = world.GetBody(manifold.b);
            const double friction = std::max(
                0.0, 0.5 * (bodyA.dynamicFriction + bodyB.dynamicFriction));
            for (const Contact& contact : manifold.contacts) {
                ++out.contactPointCount;
                if (!std::isfinite(contact.penetration)
                    || contact.penetration > settings.maxContactPenetration) {
                    out.maxContactPenetration = contact.penetration;
                    out.failureReason = ProximalFailureReason::ExtremePenetration;
                    return false;
                }
                out.maxContactPenetration =
                    std::max(out.maxContactPenetration, contact.penetration);
                const std::uint64_t contactId = PersistentContactId(manifold, contact);
                if (!seenContactIds.insert(contactId).second) {
                    ++out.duplicateContactCount;
                }
                pinocchio::JointIndex joint1 = 0;
                pinocchio::JointIndex joint2 = 0;
                Vec3 normal = manifold.normal;
                if (aRobot && bRobot) {
                    joint1 = aIt->second;
                    joint2 = bIt->second;
                } else {
                    joint2 = aRobot ? aIt->second : bIt->second;
                    normal = aRobot ? -1.0 * manifold.normal : manifold.normal;
                }
                const bool coincident = std::any_of(
                    acceptedContactGeometry.begin(),
                    acceptedContactGeometry.end(),
                    [&](const AcceptedContactGeometry& accepted) {
                        return accepted.joint1 == joint1
                            && accepted.joint2 == joint2
                            && LengthSquared(accepted.point - contact.point) <= 1.0e-10
                            && Dot(accepted.normal, normal) >= 0.999999;
                    });
                if (coincident) {
                    ++out.duplicateContactCount;
                    continue;
                }
                acceptedContactGeometry.push_back({joint1, joint2, contact.point, normal});

                std::size_t legIndex = 6;
                Vec3 pointVelocity{};
                Vec3 tibiaVelocity{};
                Vec3 spinVelocity{};
                Vec3 footVelocity{};
                if (aRobot != bRobot) {
                    const std::uint32_t robotBodyId = aRobot ? manifold.a : manifold.b;
                    const auto legIt = impl_->tibiaBodyToLeg.find(robotBodyId);
                    if (legIt != impl_->tibiaBodyToLeg.end()) {
                        legIndex = legIt->second;
                    }
                    const Body& robotBody = aRobot ? bodyA : bodyB;
                    const Vec3 r = contact.point - robotBody.position;
                    const Vec3 spin = Cross(robotBody.angularVelocity, r);
                    const Vec3 footPos = TibiaFootWorldPosition(robotBody);
                    pointVelocity = robotBody.velocity + spin;
                    tibiaVelocity = robotBody.velocity;
                    spinVelocity = spin;
                    footVelocity = robotBody.velocity
                        + Cross(robotBody.angularVelocity, footPos - robotBody.position);
                }
                pendingContacts.push_back({
                    {joint1, joint2, contact.point, normal},
                    contactId,
                    friction,
                    std::max(0.0, 0.5 * (bodyA.restitution + bodyB.restitution)),
                    contact.penetration,
                    legIndex,
                    pointVelocity,
                    tibiaVelocity,
                    spinVelocity,
                    footVelocity});
            }
        }
        if (omitContactId != 0) {
            pendingContacts.erase(
                std::remove_if(
                    pendingContacts.begin(),
                    pendingContacts.end(),
                    [omitContactId](const PendingContact& pending) {
                        return pending.id == omitContactId;
                    }),
                pendingContacts.end());
        }
        if (minPenetration > 0.0) {
            pendingContacts.erase(
                std::remove_if(
                    pendingContacts.begin(),
                    pendingContacts.end(),
                    [minPenetration](const PendingContact& pending) {
                        return pending.penetration < minPenetration;
                    }),
                pendingContacts.end());
        }

        std::sort(
            pendingContacts.begin(),
            pendingContacts.end(),
            [contactOrderSeed = impl_->contactOrderSeed](
                const PendingContact& lhs, const PendingContact& rhs) {
                const std::uint64_t lhsOrder = ContactOrderKey(lhs.id, contactOrderSeed);
                const std::uint64_t rhsOrder = ContactOrderKey(rhs.id, contactOrderSeed);
                if (lhsOrder != rhsOrder) return lhsOrder < rhsOrder;
                if (lhs.id != rhs.id) return lhs.id < rhs.id;
                if (lhs.geometry.joint1 != rhs.geometry.joint1) {
                    return lhs.geometry.joint1 < rhs.geometry.joint1;
                }
                if (lhs.geometry.joint2 != rhs.geometry.joint2) {
                    return lhs.geometry.joint2 < rhs.geometry.joint2;
                }
                if (lhs.geometry.point.x != rhs.geometry.point.x) {
                    return lhs.geometry.point.x < rhs.geometry.point.x;
                }
                if (lhs.geometry.point.y != rhs.geometry.point.y) {
                    return lhs.geometry.point.y < rhs.geometry.point.y;
                }
                return lhs.geometry.point.z < rhs.geometry.point.z;
            });
        for (const PendingContact& pending : pendingContacts) {
            contactIds.push_back(pending.id);
            contactFrictions.push_back(pending.friction);
            contactRestitutions.push_back(pending.restitution);
            contactPenetrations.push_back(pending.penetration);
        }
        {
            std::unordered_set<pinocchio::JointIndex> robotJoints;
            for (const PendingContact& pending : pendingContacts) {
                const pinocchio::JointIndex robotJoint =
                    pending.geometry.joint1 != 0
                        ? pending.geometry.joint1
                        : pending.geometry.joint2;
                if (robotJoint != 0) {
                    robotJoints.insert(robotJoint);
                }
            }
            out.uniqueRobotJointCount = robotJoints.size();
        }

        if (!pendingContacts.empty()) {
            out.contactConstraintCount = pendingContacts.size();
            for (const std::uint64_t contactId : contactIds) {
                out.contactSetSignature ^= contactId + 0x9e3779b97f4a7c15ULL
                    + (out.contactSetSignature << 6U)
                    + (out.contactSetSignature >> 2U);
            }
            impl_->lastContactSetSignature = out.contactSetSignature;
            impl_->haveLastContactSetSignature = true;

            const std::unordered_set<std::uint64_t> currentContactIds(
                contactIds.begin(), contactIds.end());
            for (auto it = impl_->contactWarmStarts.begin();
                 it != impl_->contactWarmStarts.end();) {
                if (currentContactIds.find(it->first) == currentContactIds.end()) {
                    it = impl_->contactWarmStarts.erase(it);
                    ++impl_->totalWarmStartResets;
                } else {
                    ++it;
                }
            }

            const auto makePointModel = [&](const PendingContact& pending) {
                const pinocchio::SE3 contactWorld(
                    ContactFrameRotation(pending.geometry.normal),
                    ToEigen(pending.geometry.point));
                const pinocchio::SE3 placement1 = pending.geometry.joint1 == 0
                    ? contactWorld
                    : impl_->data.oMi[pending.geometry.joint1].inverse() * contactWorld;
                const pinocchio::SE3 placement2 = pending.geometry.joint2 == 0
                    ? contactWorld
                    : impl_->data.oMi[pending.geometry.joint2].inverse() * contactWorld;
                pinocchio::PointContactConstraintModel pointModel(
                    impl_->model,
                    pending.geometry.joint1,
                    placement1,
                    pending.geometry.joint2,
                    placement2);
                pointModel.setFriction(pending.friction);
                return pinocchio::ConstraintModel(pointModel);
            };

            const bool topologyChanged = contactIds != impl_->contactTopologyIds
                || impl_->contactDelassus == nullptr
                || impl_->contactDelassusRegularization != settings.contactRegularization;
            const bool sameConstraintCount = impl_->contactDelassus != nullptr
                && impl_->contactConstraintModels.size() == pendingContacts.size()
                && impl_->contactDelassusRegularization == settings.contactRegularization;
            if (topologyChanged && !sameConstraintCount) {
                impl_->contactSolverResult.reset();
                impl_->contactSolver.reset();
                impl_->contactConstraintModels.clear();
                impl_->contactConstraintDatas.clear();
                impl_->contactConstraintModels.reserve(pendingContacts.size());
                impl_->contactConstraintDatas.reserve(pendingContacts.size());
                for (const PendingContact& pending : pendingContacts) {
                    impl_->contactConstraintModels.push_back(makePointModel(pending));
                    impl_->contactConstraintDatas.push_back(
                        impl_->contactConstraintModels.back().createData());
                }
                impl_->contactTopologyIds = contactIds;
                impl_->contactDelassusRegularization = settings.contactRegularization;
                impl_->contactDelassus = std::make_unique<RigidDelassus>(
                    std::cref(impl_->model),
                    std::ref(impl_->data),
                    std::cref(impl_->contactConstraintModels),
                    std::cref(impl_->contactConstraintDatas),
                    settings.contactRegularization);
            } else {
                for (std::size_t i = 0; i < pendingContacts.size(); ++i) {
                    impl_->contactConstraintModels[i] = makePointModel(pendingContacts[i]);
                    if (topologyChanged) {
                        impl_->contactConstraintDatas[i] =
                            impl_->contactConstraintModels[i].createData();
                    }
                }
                impl_->contactTopologyIds = contactIds;
            }

            PinConstraintModels& constraintModels = impl_->contactConstraintModels;
            PinConstraintDatas& constraintDatas = impl_->contactConstraintDatas;
            RigidDelassus& delassus = *impl_->contactDelassus;
            for (std::size_t i = 0; i < constraintModels.size(); ++i) {
                constraintModels[i].calc(impl_->model, impl_->data, constraintDatas[i]);
            }
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            const auto delassusStart = StepClock::now();
            delassus.compute();
            Eigen::VectorXd contactScale;
            double solverFeasibilityToleranceScale = 1.0;
            const bool implicitContactOp = haveImplicitH;
            if (implicitContactOp) {
                const Eigen::MatrixXd contactJacobian = MaterializeConstraintJacobian(
                    impl_->model,
                    impl_->data,
                    constraintModels,
                    constraintDatas);
                implicitGUndamped = contactJacobian
                    * implicitSolve.Hldlt.solve(contactJacobian.transpose());
                Eigen::MatrixXd contactMatrix = implicitGUndamped;
                contactMatrix.diagonal().array() += settings.contactRegularization;
                impl_->contactDenseDelassus.rebuild(contactMatrix);
            } else if (impl_->contactPrecondition) {
                contactScale = Eigen::VectorXd::Ones(delassus.rows());
                Eigen::MatrixXd contactMatrix = delassus.undampedMatrix(true);
                for (Eigen::Index offset = 0; offset < contactMatrix.rows(); offset += 3) {
                    const double meanDiagonal = contactMatrix.diagonal()
                        .segment<3>(offset)
                        .mean();
                    const double scale = 1.0 / std::sqrt(std::max(meanDiagonal, 1.0e-12));
                    contactScale.segment<3>(offset).setConstant(scale);
                }
                contactMatrix = contactScale.asDiagonal()
                    * contactMatrix
                    * contactScale.asDiagonal();
                impl_->contactDenseDelassus.rebuild(contactMatrix);
                constexpr double kPhysicalResidualSafetyFactor = 0.05;
                solverFeasibilityToleranceScale = kPhysicalResidualSafetyFactor
                    * std::min(
                        contactScale.minCoeff(), 1.0 / contactScale.maxCoeff());
            } else if (impl_->denseAdmm) {
                // The rigid-body operator remains the source of the
                // articulated Delassus matrix and final generalized impulse.
                // Materialising its small contact-space matrix makes repeated
                // ADMM products dense matrix-vector operations.
                impl_->contactDenseDelassus.rebuild(delassus, true);
            }
            out.delassusTimeMs += elapsedMs(delassusStart);
            constraintAssemblyStart = StepClock::now();

            std::array<double, 6> sphereFootX{};
            std::array<double, 6> sphereFootZ{};
            std::array<double, 6> sphereFootPosVx{};
            std::array<double, 6> sphereFootPosVz{};
            for (const auto& tibiaAndLeg : impl_->tibiaBodyToLeg) {
                const std::size_t leg = tibiaAndLeg.second;
                if (leg >= 6) {
                    continue;
                }
                const Vec3 footPos = TibiaFootWorldPosition(world.GetBody(tibiaAndLeg.first));
                sphereFootX[leg] = footPos.x;
                sphereFootZ[leg] = footPos.z;
                if (impl_->havePrevFootX[leg] && subDt > 0.0) {
                    sphereFootPosVx[leg] = (footPos.x - impl_->prevFootX[leg]) / subDt;
                    sphereFootPosVz[leg] = (footPos.z - impl_->prevFootZ[leg]) / subDt;
                }
                impl_->prevFootX[leg] = footPos.x;
                impl_->prevFootZ[leg] = footPos.z;
                impl_->havePrevFootX[leg] = true;
            }

            Eigen::VectorXd drift(static_cast<Eigen::Index>(3U * constraintModels.size()));
            pinocchio::evalConstraintJacobianMatrixProduct(
                impl_->model,
                impl_->data,
                constraintModels,
                constraintDatas,
                vNew,
                drift,
                pinocchio::SetTo());
            const Eigen::VectorXd freeContactVelocity = drift;
            std::vector<double> penetrationCorrections(constraintModels.size(), 0.0);
            const ContactSolverConfig& contactSettings = world.GetContactSolverConfig();
            for (std::size_t i = 0; i < constraintModels.size(); ++i) {
                const Eigen::Index normalIndex = static_cast<Eigen::Index>(3U * i + 2U);
                const double correction = std::min(
                    std::max(0.0, contactSettings.penetrationBiasMaxSpeed),
                    std::max(0.0, contactSettings.penetrationBiasFactor)
                        * std::max(0.0, contactPenetrations[i] - contactSettings.penetrationSlop)
                        / subDt);
                penetrationCorrections[i] = correction;
                drift[normalIndex] -= correction;
                const double incomingNormalSpeed = drift[normalIndex];
                const double restitutionCutoff = std::max(
                    contactSettings.restitutionVelocityCutoff,
                    settings.restitutionVelocityCutoff);
                if (incomingNormalSpeed < -restitutionCutoff) {
                    drift[normalIndex] += contactRestitutions[i] * incomingNormalSpeed;
                }
            }
            for (std::size_t i = 0; i < pendingContacts.size(); ++i) {
                const PendingContact& pending = pendingContacts[i];
                if (pending.legIndex >= 6) {
                    continue;
                }
                const Eigen::Matrix3d rotation =
                    ContactFrameRotation(pending.geometry.normal);
                const Eigen::Vector3d worldSlipTx = rotation.transpose()
                    * ToEigen(pending.pointVelocity);
                out.legPinocchioDriftTx[pending.legIndex] =
                    drift[static_cast<Eigen::Index>(3U * i)];
                out.legWorldSlipTx[pending.legIndex] = worldSlipTx.x();
                out.legWorldSlipTy[pending.legIndex] = worldSlipTx.y();
                out.legTibiaVx[pending.legIndex] = pending.tibiaVelocity.x;
                out.legSpinVx[pending.legIndex] = pending.spinVelocity.x;
                out.legT0x[pending.legIndex] = rotation.col(0).x();
                out.legFootVx[pending.legIndex] = pending.footVelocity.x;
                out.legFootX[pending.legIndex] = sphereFootX[pending.legIndex];
                out.legFootPosVx[pending.legIndex] = sphereFootPosVx[pending.legIndex];
                out.legFootVz[pending.legIndex] = pending.footVelocity.z;
                out.legFootZ[pending.legIndex] = sphereFootZ[pending.legIndex];
                out.legFootPosVz[pending.legIndex] = sphereFootPosVz[pending.legIndex];
            }
            Eigen::VectorXd scaledDrift;
            if (impl_->contactPrecondition) {
                scaledDrift = contactScale.cwiseProduct(drift);
            }
            const Eigen::VectorXd& solverDrift =
                impl_->contactPrecondition ? scaledDrift : drift;

            Eigen::VectorXd warm = Eigen::VectorXd::Zero(drift.size());
            Eigen::VectorXd warmVelocity = Eigen::VectorXd::Zero(drift.size());
            const Vec3 gravity = world.GetGravity();
            const std::size_t supportingContactCount = static_cast<std::size_t>(
                std::count_if(
                    pendingContacts.begin(),
                    pendingContacts.end(),
                    [&](const PendingContact& pending) {
                        return Dot(gravity, pending.geometry.normal) < -1.0e-6;
                    }));
            double existingNormalImpulseSum = 0.0;
            std::size_t existingNormalImpulseCount = 0;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const auto found = impl_->contactWarmStarts.find(contactIds[i]);
                if (found == impl_->contactWarmStarts.end()) {
                    continue;
                }
                Eigen::Vector3d impulse = found->second.impulse;
                Eigen::Vector3d velocity = found->second.velocity;
                if (found->second.haveFrame) {
                    const Eigen::Matrix3d newFrame =
                        ContactFrameRotation(pendingContacts[i].geometry.normal);
                    const double frameRotation = ContactFrameRotationAngleRad(
                        found->second.frame, newFrame);
                    if (transportWarmStarts && !transportContactWarmStart(
                            found->second.frame, newFrame, impulse, velocity)) {
                        ++impl_->totalWarmStartResets;
                    }
                    static std::uint64_t warmStartTransportLogs = 0;
                    if (frameRotation > 5.0e-2 && warmStartTransportLogs < 8) {
                        ++warmStartTransportLogs;
                        std::cerr << "[proximal-warm-start-transport] id="
                                  << contactIds[i]
                                  << " frame_rot_rad=" << frameRotation
                                  << (transportWarmStarts ? " (transported world impulse/velocity)\n"
                                                         : " (logged only; contact-frame λ reused)\n");
                    }
                }
                if (found->second.dt > 0.0) {
                    impulse *= subDt / found->second.dt;
                }
                impulse[2] = std::max(0.0, impulse[2]);
                const double tangential = impulse.head<2>().norm();
                const double limit = contactFrictions[i] * impulse[2];
                if (tangential > limit && tangential > 0.0) {
                    impulse.head<2>() *= limit / tangential;
                }
                warm.segment<3>(static_cast<Eigen::Index>(3U * i)) = impulse;
                warmVelocity.segment<3>(static_cast<Eigen::Index>(3U * i)) = velocity;
                existingNormalImpulseSum += impulse[2];
                ++existingNormalImpulseCount;
            }
            const double newContactNormalGuess = existingNormalImpulseCount != 0
                ? existingNormalImpulseSum / static_cast<double>(existingNormalImpulseCount)
                : 0.0;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                if (impl_->contactWarmStarts.find(contactIds[i])
                    != impl_->contactWarmStarts.end()) {
                    continue;
                }
                if (existingNormalImpulseCount != 0) {
                    warm[static_cast<Eigen::Index>(3U * i + 2U)] = newContactNormalGuess;
                    continue;
                }
                if (supportingContactCount == 0) {
                    continue;
                }
                const double gravityNormalAcceleration = std::max(
                    0.0, -Dot(gravity, pendingContacts[i].geometry.normal));
                warm[static_cast<Eigen::Index>(3U * i + 2U)] =
                    impl_->totalRobotMass * gravityNormalAcceleration * subDt
                    / static_cast<double>(supportingContactCount);
            }
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            out.contactSetupTimeMs += elapsedMs(contactSetupStart);

            pinocchio::ADMMSolverSettings solverSettings;
            // Pinocchio numbers iterations from zero and loops while
            // `iterations <= max_iterations`; subtract one so the public
            // configuration remains a true maximum solve count.
            const int iterationCap = iterationOverride > 0
                ? iterationOverride
                : settings.maxIterations;
            solverSettings.max_iterations = static_cast<std::size_t>(
                std::max(1, iterationCap) - 1);
            // Standing contacts converge in a few iterations at 1e-8 and hold
            // height. Sliding (walk) contacts often need the looser ADMM stop:
            // continuing to 1e-8 hits the cap and the last iterate is worse
            // than the 1e-3 NCP-feasible one. Use tangential free speed to
            // pick the stop without a gait-mode signal from the server.
            constexpr double kSlidingContactSpeed = 2.0e-2;
            double maxTangentialFreeSpeed = 0.0;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                maxTangentialFreeSpeed = std::max(
                    maxTangentialFreeSpeed,
                    drift.segment<2>(static_cast<Eigen::Index>(3U * i)).norm());
            }
            const double admmAbsoluteTolerance =
                maxTangentialFreeSpeed > kSlidingContactSpeed
                    ? std::max(settings.absoluteTolerance, settings.ncpAbsoluteTolerance)
                    : settings.absoluteTolerance;
            solverSettings.absolute_feasibility_tol = admmAbsoluteTolerance
                * solverFeasibilityToleranceScale;
            solverSettings.relative_feasibility_tol = settings.relativeTolerance
                * solverFeasibilityToleranceScale;
            solverSettings.absolute_complementarity_tol = admmAbsoluteTolerance;
            solverSettings.relative_complementarity_tol = settings.relativeTolerance;
            solverSettings.admm_update_rule = pinocchio::ADMMUpdateRule::SPECTRAL;
            solverSettings.admm_proximal_rule = pinocchio::ADMMProximalRule::MANUAL;
            solverSettings.mu_prox = settings.proximalMu;
            solverSettings.anderson_capacity = andersonCapacity;
            solverSettings.ratio_primal_dual = impl_->ratioPrimalDual;
            solverSettings.tau = impl_->admmTau;
            solverSettings.spectral_rho_power_init = impl_->spectralRhoPowerInit;
            solverSettings.warmstart_rho_with_previous_result = impl_->warmstartRho;
            solverSettings.solve_ncp = true;
            solverSettings.stat_record = false;
            pinocchio::ADMMSolverResult& result = impl_->contactSolverResult;
            if (result.constraintSize() != drift.size()) {
                result.resize(static_cast<std::size_t>(drift.size()));
                result.reset();
            }
            Eigen::VectorXd scaledWarm;
            Eigen::VectorXd scaledWarmVelocity;
            if (impl_->contactPrecondition) {
                scaledWarm = warm.cwiseQuotient(contactScale);
                scaledWarmVelocity = warmVelocity.cwiseProduct(contactScale);
            }
            const Eigen::VectorXd& solverWarm =
                impl_->contactPrecondition ? scaledWarm : warm;
            const Eigen::VectorXd& solverWarmVelocity =
                impl_->contactPrecondition ? scaledWarmVelocity : warmVelocity;
            result.setConstraintImpulseGuess(solverWarm);
            if (IsFinite(solverWarmVelocity)) {
                result.setConstraintVelocityGuess(solverWarmVelocity);
            }
            const auto admmStart = StepClock::now();
            const bool useDenseContactOp =
                implicitContactOp || impl_->denseAdmm || impl_->contactPrecondition;
            const bool converged = useDenseContactOp
                ? impl_->contactSolver.solve(
                    impl_->contactDenseDelassus,
                    solverDrift,
                    constraintModels,
                    constraintDatas,
                    solverSettings,
                    result)
                : impl_->contactSolver.solve(
                    delassus,
                    solverDrift,
                    constraintModels,
                    constraintDatas,
                    solverSettings,
                    result);
            out.admmTimeMs += elapsedMs(admmStart);
            out.admmRho = result.rho;
            out.iterations = std::max(out.iterations, static_cast<int>(result.iterations));
            out.primalResidual = std::max(out.primalResidual, result.primal_feasibility);
            out.dualResidual = std::max(out.dualResidual, result.dual_feasibility);
            out.complementarityResidual = std::max(
                out.complementarityResidual, result.complementarity);
            Eigen::VectorXd impulses(drift.size());
            result.retrieveConstraintImpulses(impulses);
            if (impl_->contactPrecondition) {
                impulses.array() *= contactScale.array();
            }
            const char* compliantExperimentEnv =
                std::getenv("HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT");
            const bool compliantExperiment = settings.compliantContact
                || forceCompliantContact
                || (compliantExperimentEnv != nullptr
                    && compliantExperimentEnv[0] != static_cast<char>(0));
            bool compliantConverged = false;
            double compliantProjectedResidual =
                std::numeric_limits<double>::infinity();
            Eigen::VectorXd compliantSolvedContactVelocities;
            if (compliantExperiment && IsFinite(drift)) {
                const Eigen::MatrixXd compliantMatrix = implicitContactOp
                    ? implicitGUndamped
                    : delassus.undampedMatrix(true);
                const auto compliantDoubleSetting = [](const char* name, const double fallback) {
                    const char* value = std::getenv(name);
                    if (value == nullptr || value[0] == static_cast<char>(0)) {
                        return fallback;
                    }
                    char* end = nullptr;
                    const double parsed = std::strtod(value, &end);
                    return end != value && *end == static_cast<char>(0) && std::isfinite(parsed) && parsed > 0.0
                        ? parsed
                        : fallback;
                };
                const auto compliantIntegerSetting = [](const char* name, const int fallback) {
                    const char* value = std::getenv(name);
                    if (value == nullptr || value[0] == static_cast<char>(0)) {
                        return fallback;
                    }
                    char* end = nullptr;
                    const long parsed = std::strtol(value, &end, 10);
                    return end != value && *end == static_cast<char>(0)
                        ? static_cast<int>(std::clamp(parsed, 1L, 800L))
                        : fallback;
                };
                const double normalCompliance = compliantDoubleSetting(
                    "HEXAPOD_PINOCCHIO_COMPLIANT_NORMAL_COMPLIANCE", 1.0e-5);
                const double tangentialRegularization = compliantDoubleSetting(
                    "HEXAPOD_PINOCCHIO_COMPLIANT_TANGENTIAL_REGULARIZATION", 1.0e-6);
                Eigen::MatrixXd regularized = compliantMatrix;
                for (std::size_t i = 0; i < contactIds.size(); ++i) {
                    regularized(static_cast<Eigen::Index>(3U * i + 0U), static_cast<Eigen::Index>(3U * i + 0U)) += tangentialRegularization;
                    regularized(static_cast<Eigen::Index>(3U * i + 1U), static_cast<Eigen::Index>(3U * i + 1U)) += tangentialRegularization;
                    regularized(static_cast<Eigen::Index>(3U * i + 2U), static_cast<Eigen::Index>(3U * i + 2U)) += normalCompliance;
                }
                const double maxRowSum = regularized.rowwise().lpNorm<1>().maxCoeff();
                const double projectedStep = 1.0 / std::max(maxRowSum, 1.0e-9);
                // Start from the already-bounded rigid ADMM impulse. An
                // unconstrained LDLT start can sit far outside the Coulomb
                // cone and then inject hundreds of rad/s if the projected
                // loop is accepted before it has recovered.
                Eigen::VectorXd compliantImpulse = impulses;
                if (!IsFinite(compliantImpulse) || compliantImpulse.size() != drift.size()) {
                    compliantImpulse = regularized.ldlt().solve(-drift);
                }
                if (!IsFinite(compliantImpulse)) {
                    compliantImpulse = Eigen::VectorXd::Zero(drift.size());
                }
                for (std::size_t i = 0; i < contactIds.size(); ++i) {
                    const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                    compliantImpulse.segment<3>(offset) = ProjectCoulombImpulse(
                        compliantImpulse.segment<3>(offset), contactFrictions[i]);
                }
                // Frozen reverse/turn fixtures need roughly 195--232 projected
                // iterations. The old live cap of 20 then unconditionally
                // accepted a visibly unconverged impulse, which could inject
                // hundreds of rad/s at a two-contact transition.
                const int compliantIterations = compliantIntegerSetting(
                    "HEXAPOD_PINOCCHIO_COMPLIANT_ITERATIONS", 256);
                // Live acceptance uses 1e-3, matching the production NCP floor.
                // The frozen-fixture 1e-5 KKT gate rejected physically good
                // walking steps whose dual residual was already ~1e-6.
                constexpr double kLiveProjectedTolerance = 1.0e-3;
                int completedIterations = 0;
                auto evaluateProjectedResidual = [&](const Eigen::VectorXd& lambda) {
                    const Eigen::VectorXd gradient = regularized * lambda + drift;
                    double residual = 0.0;
                    for (std::size_t i = 0; i < contactIds.size(); ++i) {
                        const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                        const Eigen::Vector3d projected = ProjectCoulombImpulse(
                            lambda.segment<3>(offset) - projectedStep * gradient.segment<3>(offset),
                            contactFrictions[i]);
                        residual = std::max(
                            residual,
                            (projected - lambda.segment<3>(offset)).lpNorm<Eigen::Infinity>()
                                / std::max(projectedStep, 1.0e-12));
                    }
                    return residual;
                };
                compliantProjectedResidual = evaluateProjectedResidual(compliantImpulse);
                for (int iteration = 0; iteration < compliantIterations; ++iteration) {
                    if (std::isfinite(compliantProjectedResidual)
                        && compliantProjectedResidual <= kLiveProjectedTolerance) {
                        break;
                    }
                    const Eigen::VectorXd gradient = regularized * compliantImpulse + drift;
                    Eigen::VectorXd candidate = compliantImpulse - projectedStep * gradient;
                    for (std::size_t i = 0; i < contactIds.size(); ++i) {
                        const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                        candidate.segment<3>(offset) = ProjectCoulombImpulse(
                            candidate.segment<3>(offset), contactFrictions[i]);
                    }
                    const double delta = (candidate - compliantImpulse).lpNorm<Eigen::Infinity>();
                    compliantImpulse = candidate;
                    completedIterations = iteration + 1;
                    compliantProjectedResidual = evaluateProjectedResidual(compliantImpulse);
                    if (delta <= 1.0e-8) {
                        break;
                    }
                }
                impulses = compliantImpulse;
                out.iterations = std::max(out.iterations, completedIterations);
                compliantSolvedContactVelocities = Eigen::VectorXd::Zero(drift.size());
                if (implicitContactOp) {
                    compliantSolvedContactVelocities =
                        implicitGUndamped * compliantImpulse + drift;
                } else {
                    delassus.applyOnTheRight(
                        compliantImpulse, compliantSolvedContactVelocities, false);
                    compliantSolvedContactVelocities += drift;
                }
                const char* traceCompliant =
                    std::getenv("HEXAPOD_PINOCCHIO_TRACE_COMPLIANT");
                if (traceCompliant != nullptr && traceCompliant[0] != static_cast<char>(0)
                    && traceCompliant[0] != '0') {
                    std::cerr << "[proximal-compliant-experiment] contacts=" << contactIds.size()
                              << " iterations=" << completedIterations
                              << " iterations_cap=" << compliantIterations
                              << " normal_compliance=" << normalCompliance
                              << " tangential_regularization=" << tangentialRegularization
                              << " projected_step=" << projectedStep
                              << " projected_residual=" << compliantProjectedResidual
                              << " peak_impulse=" << compliantImpulse.lpNorm<Eigen::Infinity>()
                              << "\n";
                }
                out.compliantProjectedResidual = compliantProjectedResidual;
            }
            if (IsFinite(impulses)) {
                Eigen::VectorXd contactVelocities(drift.size());
                if (implicitContactOp) {
                    contactVelocities = implicitGUndamped * impulses + drift;
                } else {
                    delassus.applyOnTheRight(impulses, contactVelocities, false);
                    contactVelocities += drift;
                }
                Eigen::VectorXd deSaxce = Eigen::VectorXd::Zero(drift.size());
                Eigen::VectorXd correctedVelocities = Eigen::VectorXd::Zero(drift.size());
                Eigen::VectorXd projectedDual = Eigen::VectorXd::Zero(drift.size());
                pinocchio::internal::computeDeSaxeCorrection(
                    constraintModels, constraintDatas, contactVelocities, deSaxce);
                correctedVelocities = contactVelocities + deSaxce;
                pinocchio::internal::computeDualConstraintSetProjection(
                    constraintModels, constraintDatas, correctedVelocities, projectedDual);
                out.ncpDualResidual =
                    (projectedDual - correctedVelocities).lpNorm<Eigen::Infinity>();
                pinocchio::internal::computeConicComplementarity(
                    constraintModels,
                    constraintDatas,
                    correctedVelocities,
                    impulses,
                    out.ncpComplementarityResidual);
                double worstContactResidual = -1.0;
                double worstComplementarityResidual = -1.0;
                out.tracedContactCount = std::min(contactIds.size(), out.tracedContactId.size());
                for (std::size_t i = 0; i < contactIds.size(); ++i) {
                    const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                    const Eigen::Vector3d impulse = impulses.segment<3>(offset);
                    const Eigen::Vector3d corrected = correctedVelocities.segment<3>(offset);
                    const double coneResidual =
                        (impulse - ProjectCoulombImpulse(impulse, contactFrictions[i])).norm();
                    const double dualResidual = (projectedDual - correctedVelocities)
                                                    .segment<3>(offset)
                                                    .lpNorm<Eigen::Infinity>();
                    const double complementarityResidual = std::abs(impulse.dot(corrected));
                    out.coneResidual = std::max(out.coneResidual, coneResidual);
                    const double contactResidual = std::max(coneResidual, dualResidual);
                    if (contactResidual > worstContactResidual) {
                        worstContactResidual = contactResidual;
                        out.worstContactId = contactIds[i];
                    }
                    if (complementarityResidual > worstComplementarityResidual) {
                        worstComplementarityResidual = complementarityResidual;
                        out.worstComplementarityContactId = contactIds[i];
                    }
                    if (i < out.tracedContactCount) {
                        const PendingContact& pending = pendingContacts[i];
                        out.tracedContactId[i] = pending.id;
                        out.tracedContactLeg[i] =
                            pending.legIndex < 6
                                ? static_cast<std::uint8_t>(pending.legIndex)
                                : static_cast<std::uint8_t>(255);
                        out.tracedContactJoint[i] = static_cast<int>(
                            pending.geometry.joint1 != 0
                                ? pending.geometry.joint1
                                : pending.geometry.joint2);
                        out.tracedContactComp[i] = complementarityResidual;
                        out.tracedContactDual[i] = dualResidual;
                        out.tracedContactCone[i] = coneResidual;
                        out.tracedContactPen[i] = pending.penetration;
                    }
                }
            }
            Eigen::VectorXd solvedContactVelocities(drift.size());
            if (compliantExperiment) {
                solvedContactVelocities = compliantSolvedContactVelocities;
            } else {
                result.retrieveConstraintVelocities(solvedContactVelocities);
                if (impl_->contactPrecondition) {
                    solvedContactVelocities.array() /= contactScale.array();
                }
            }
            const double driftScale = drift.lpNorm<Eigen::Infinity>();
            const double impulseScale = IsFinite(impulses)
                ? impulses.lpNorm<Eigen::Infinity>()
                : std::numeric_limits<double>::infinity();
            const double ncpAbsoluteTolerance = ncpAcceptAbsoluteOverride > 0.0
                ? ncpAcceptAbsoluteOverride
                : std::max(settings.absoluteTolerance, settings.ncpAbsoluteTolerance);
            const double ncpDualTolerance = ncpAbsoluteTolerance
                + settings.relativeTolerance * driftScale;
            const double ncpComplementarityTolerance = ncpAbsoluteTolerance
                + settings.relativeTolerance * impulseScale * driftScale;
            const double coneTolerance = ncpAbsoluteTolerance
                + settings.relativeTolerance * impulseScale;
            bool physicallyConverged = IsFinite(impulses)
                && std::isfinite(out.ncpDualResidual)
                && std::isfinite(out.ncpComplementarityResidual)
                && std::isfinite(out.coneResidual)
                && out.ncpDualResidual <= ncpDualTolerance
                && out.ncpComplementarityResidual <= ncpComplementarityTolerance
                && out.coneResidual <= coneTolerance;
            if (compliantExperiment) {
                // Reject only unsafe iterates: non-finite values, or an
                // impulse large enough to reproduce the two-contact blow-up
                // (hundreds of rad/s). Walking peaks sit near 0.06 N·s.
                constexpr double kMaxCompliantImpulse = 1.0;
                constexpr double kLiveProjectedTolerance = 1.0e-3;
                const double peakImpulse = IsFinite(impulses)
                    ? impulses.lpNorm<Eigen::Infinity>()
                    : std::numeric_limits<double>::infinity();
                compliantConverged = IsFinite(impulses)
                    && std::isfinite(compliantProjectedResidual)
                    && peakImpulse <= kMaxCompliantImpulse
                    && (compliantProjectedResidual <= kLiveProjectedTolerance
                        || physicallyConverged);
                physicallyConverged = compliantConverged;
            }
            out.admmConverged = compliantExperiment ? compliantConverged : converged;
            out.ncpPhysicallyConverged = physicallyConverged;
            bool accepted = compliantExperiment
                ? compliantConverged
                : (impl_->contactPrecondition
                    ? physicallyConverged
                    : converged || physicallyConverged);
            if (!accepted
                || !std::isfinite(result.primal_feasibility)
                || !std::isfinite(result.dual_feasibility)
                || !std::isfinite(result.complementarity)) {
                if ((traceFailures
                    && iterationOverride == 48
                    && omitContactId == 0
                    && minPenetration == 0.0) || (std::getenv("HEXAPOD_PINOCCHIO_CONTACT_SNAPSHOT_PATH") != nullptr)) {
                    const Eigen::MatrixXd contactMatrix = delassus.undampedMatrix(true);
                    if (const char* snapshotPath = std::getenv("HEXAPOD_PINOCCHIO_CONTACT_SNAPSHOT_PATH");
                        snapshotPath != nullptr && snapshotPath[0] != '\0'
                        && !impl_->contactSnapshotCaptured) {
                        try {
                            Eigen::MatrixXd jacobian(
                                static_cast<Eigen::Index>(3U * constraintModels.size()),
                                impl_->model.nv);
                            for (Eigen::Index column = 0; column < impl_->model.nv; ++column) {
                                Eigen::VectorXd basis = Eigen::VectorXd::Zero(impl_->model.nv);
                                basis[column] = 1.0;
                                Eigen::VectorXd product = Eigen::VectorXd::Zero(jacobian.rows());
                                pinocchio::evalConstraintJacobianMatrixProduct(
                                    impl_->model, impl_->data, constraintModels, constraintDatas,
                                    basis, product, pinocchio::SetTo());
                                jacobian.col(column) = product;
                            }
                            Eigen::MatrixXd mass = pinocchio::crba(
                                impl_->model, impl_->contactInertiaData, q,
                                pinocchio::Convention::WORLD);
                            mass.triangularView<Eigen::StrictlyLower>() =
                                mass.transpose().triangularView<Eigen::StrictlyLower>();
                            const Eigen::MatrixXd massInverse = mass.ldlt().solve(
                                Eigen::MatrixXd::Identity(impl_->model.nv, impl_->model.nv));
                            const Eigen::MatrixXd denseMatrix =
                                jacobian * massInverse * jacobian.transpose();
                            pinocchio::crba(impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
                            impl_->data.M.triangularView<Eigen::StrictlyLower>() =
                                impl_->data.M.transpose().triangularView<Eigen::StrictlyLower>();
                            pinocchio::ConstraintCholeskyDecomposition cholesky(
                                impl_->model, impl_->data, constraintModels, constraintDatas,
                                settings.contactRegularization);
                            cholesky.compute(
                                impl_->model, impl_->data, constraintModels, constraintDatas,
                                settings.contactRegularization);
                            const Eigen::MatrixXd oracleMatrix =
                                cholesky.getDelassusOperatorCholeskyExpression().matrix(false, true);
                            Eigen::MatrixXd wrenchMap(6, jacobian.rows());
                            wrenchMap.setZero();
                            const Eigen::Vector3d chassisPosition = ToEigen(
                                world.GetBody(impl_->bodies.front().bodyId).position);
                            for (std::size_t contact = 0; contact < pendingContacts.size(); ++contact) {
                                const Eigen::Matrix3d frame = ContactFrameRotation(
                                    pendingContacts[contact].geometry.normal);
                                const Eigen::Vector3d lever = ToEigen(
                                    pendingContacts[contact].geometry.point) - chassisPosition;
                                for (int axis = 0; axis < 3; ++axis) {
                                    const Eigen::Vector3d force = frame.col(axis);
                                    const Eigen::Vector3d torque = lever.cross(force);
                                    wrenchMap.block<3, 1>(0, 3 * contact + axis) = force;
                                    wrenchMap.block<3, 1>(3, 3 * contact + axis) = torque;
                                }
                            }
                            const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> denseSpectrum(
                                denseMatrix);
                            const Eigen::JacobiSVD<Eigen::MatrixXd> jacobianSvd(
                                jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
                            const Eigen::JacobiSVD<Eigen::MatrixXd> wrenchSvd(
                                wrenchMap, Eigen::ComputeThinU | Eigen::ComputeThinV);
                            const auto safe = [](double value) {
                                return std::isfinite(value) ? value : 0.0;
                            };
                            const auto writeVector = [&](std::ostream& stream,
                                                         const Eigen::VectorXd& vector) {
                                stream << '[';
                                for (Eigen::Index i = 0; i < vector.size(); ++i) {
                                    if (i != 0) stream << ',';
                                    stream << safe(vector[i]);
                                }
                                stream << ']';
                            };
                            const auto writeMatrix = [&](std::ostream& stream,
                                                        const Eigen::MatrixXd& matrix) {
                                stream << '[';
                                for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
                                    if (row != 0) stream << ',';
                                    writeVector(stream, matrix.row(row).transpose());
                                }
                                stream << ']';
                            };
                            std::ofstream fixture(snapshotPath, std::ios::trunc);
                            if (!fixture) {
                                throw std::runtime_error("cannot open snapshot path");
                            }
                            fixture << std::setprecision(17);
                            fixture << "{\"schema_version\":1,\"model\":{";
                            fixture << "\"nq\":" << impl_->model.nq
                                    << ",\"nv\":" << impl_->model.nv
                                    << ",\"contact_count\":" << pendingContacts.size()
                                    << "},\"step\":{";
                            fixture << "\"dt\":" << subDt << ",\"q\":";
                            writeVector(fixture, q);
                            fixture << ",\"v\":";
                            writeVector(fixture, v);
                            fixture << ",\"tau\":";
                            writeVector(fixture, tau);
                            fixture << "},\"settings\":{";
                            fixture << "\"max_iterations\":" << settings.maxIterations
                                    << ",\"proximal_mu\":" << settings.proximalMu
                                    << ",\"absolute_tolerance\":" << settings.absoluteTolerance
                                    << ",\"relative_tolerance\":" << settings.relativeTolerance
                                    << ",\"contact_regularization\":"
                                    << settings.contactRegularization << "},\"contacts\":[";
                            for (std::size_t i = 0; i < pendingContacts.size(); ++i) {
                                if (i != 0) fixture << ',';
                            const Eigen::IOFormat jsonVectorFormat(17, Eigen::DontAlign, ",", ",");
                                const PendingContact& pending = pendingContacts[i];
                                fixture << "{\"id\":" << pending.id
                                        << ",\"leg\":" << pending.legIndex
                                        << ",\"joint1\":" << pending.geometry.joint1
                                        << ",\"joint2\":" << pending.geometry.joint2
                                        << ",\"friction\":" << pending.friction
                                        << ",\"restitution\":" << pending.restitution
                                        << ",\"penetration\":" << pending.penetration
                                        << ",\"point\":[" << pending.geometry.point.x << ','
                                        << pending.geometry.point.y << ',' << pending.geometry.point.z
                                        << "],\"normal\":[" << pending.geometry.normal.x << ','
                                        << pending.geometry.normal.y << ',' << pending.geometry.normal.z
                                        << "],\"free_velocity\":["
                                        << freeContactVelocity.segment<3>(3 * i).transpose().format(jsonVectorFormat)
                                        << "],\"biased_velocity\":["
                                        << drift.segment<3>(3 * i).transpose().format(jsonVectorFormat)
                                        << "],\"impulse\":["
                                        << impulses.segment<3>(3 * i).transpose().format(jsonVectorFormat) << "]}";
                            }
                            fixture << "],\"jacobian\":";
                            writeMatrix(fixture, jacobian);
                            fixture << ",\"delassus_articulated\":";
                            writeMatrix(fixture, contactMatrix);
                            fixture << ",\"delassus_dense\":";
                            writeMatrix(fixture, denseMatrix);
                            fixture << ",\"delassus_cholesky\":";
                            writeMatrix(fixture, oracleMatrix);
                            fixture << ",\"wrench_map\":";
                            writeMatrix(fixture, wrenchMap);
                            fixture << ",\"solver_result\":{";
                            fixture << "\"accepted\":" << (accepted ? "true" : "false")
                                    << ",\"admm_converged\":" << (converged ? "true" : "false")
                                    << ",\"iterations\":" << result.iterations
                                    << ",\"primal\":" << safe(result.primal_feasibility)
                                    << ",\"dual\":" << safe(result.dual_feasibility)
                                    << ",\"complementarity\":" << safe(result.complementarity)
                                    << ",\"ncp_dual\":" << safe(out.ncpDualResidual)
                                    << ",\"ncp_complementarity\":"
                                    << safe(out.ncpComplementarityResidual)
                                    << ",\"cone\":" << safe(out.coneResidual)
                                    << ",\"failure_reason\":"
                                    << static_cast<int>(ProximalFailureReason::SolverNotConverged)
                                    << "},\"audit\":{";
                            fixture << "\"dense_articulated_max_abs\":"
                                    << safe((denseMatrix - contactMatrix).cwiseAbs().maxCoeff())
                                    << ",\"dense_cholesky_max_abs\":"
                                    << safe((denseMatrix - oracleMatrix).cwiseAbs().maxCoeff())
                                    << ",\"articulated_cholesky_max_abs\":"
                                    << safe((contactMatrix - oracleMatrix).cwiseAbs().maxCoeff());
                            if (denseSpectrum.info() == Eigen::Success) {
                                fixture << ",\"dense_eigenvalues\":";
                                writeVector(fixture, denseSpectrum.eigenvalues());
                            }
                            fixture << ",\"jacobian_singular_values\":";
                            writeVector(fixture, jacobianSvd.singularValues());
                            fixture << ",\"wrench_singular_values\":";
                            writeVector(fixture, wrenchSvd.singularValues());
                            fixture << "}}\n";
                            fixture.close();
                            impl_->contactSnapshotCaptured = true;
                            std::cerr << "[proximal-contact-snapshot] path=" << snapshotPath
                                      << " contacts=" << pendingContacts.size()
                                      << " dense_articulated_max_abs="
                                      << (denseMatrix - contactMatrix).cwiseAbs().maxCoeff()
                                      << "\n";
                        } catch (const std::exception& error) {
                            std::cerr << "[proximal-contact-snapshot] failed: "
                                      << error.what() << "\n";
                        }
                    }
                    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(contactMatrix);
                    double minEigenvalue = std::numeric_limits<double>::quiet_NaN();
                    double maxEigenvalue = std::numeric_limits<double>::quiet_NaN();
                    double condition = std::numeric_limits<double>::infinity();
                    if (spectrum.info() == Eigen::Success && spectrum.eigenvalues().size() != 0) {
                        minEigenvalue = spectrum.eigenvalues().minCoeff();
                        maxEigenvalue = spectrum.eigenvalues().maxCoeff();
                        if (minEigenvalue > 0.0) {
                            condition = maxEigenvalue / minEigenvalue;
                        }
                    }
                    std::cerr << "[proximal-held-system] eig_min=" << minEigenvalue
                              << " eig_max=" << maxEigenvalue
                              << " condition=" << condition
                              << " sub_dt=" << subDt
                              << " rows=" << contactMatrix.rows()
                              << " contacts=";
                    for (std::size_t i = 0; i < contactIds.size(); ++i) {
                        const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                        const Eigen::Vector3d free = freeContactVelocity.segment<3>(offset);
                        const Eigen::Vector3d biased = drift.segment<3>(offset);
                        const Eigen::Vector3d impulse = impulses.segment<3>(offset);
                        const std::size_t leg = pendingContacts[i].legIndex;
                        std::cerr << (i == 0 ? "[" : ",")
                                  << contactIds[i] << ':'
                                  << static_cast<unsigned>(leg) << ':'
                                  << pendingContacts[i].penetration << ':'
                                  << (leg < loadBearingLegs.size() && loadBearingLegs[leg] ? 1 : 0)
                                  << ':'
                                  << (leg < impl_->lastLegNormalImpulse.size()
                                          ? impl_->lastLegNormalImpulse[leg]
                                          : 0.0)
                                  << ':'
                                  << free.x() << ':' << free.y() << ':' << free.z() << ':'
                                  << penetrationCorrections[i] << ':' << biased.z() << ':'
                                  << impulse.x() << ':' << impulse.y() << ':' << impulse.z() << ':'
                                  << contactMatrix(offset + 2, offset + 2);
                        if (leg < 6) {
                            for (std::size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
                                const std::size_t wire = 3U * leg + jointOffset;
                                const pinocchio::JointIndex joint = impl_->wireJoints[wire];
                                const Eigen::Index qi = impl_->model.joints[joint].idx_q();
                                const Eigen::Index vi = impl_->model.joints[joint].idx_v();
                                const double targetRate = impl_->haveCommandedServoTargets
                                    ? std::remainder(
                                          activeServoTargets[wire]
                                              - impl_->commandedServoTargets[wire],
                                          6.28318530717958647692) / subDt
                                    : 0.0;
                                const double error = std::remainder(
                                    activeServoTargets[wire]
                                        - impl_->wireZeroAngles[wire] - q[qi],
                                    6.28318530717958647692);
                                std::cerr << ':' << targetRate << ':' << v[vi] << ':' << error;
                            }
                        }
                    }
                    std::cerr << "]\n";
                }
                // A bounded, non-converged iterate is still useful as a retry
                // initial guess. Preserve only contacts other than the one
                // with the worst residual; no impulse from this solve is
                // applied to the robot state. Unconverged compliant iterates
                // can be hundreds of rad/s too large, so they stay cold.
                if (IsFinite(impulses) && IsFinite(solvedContactVelocities)
                    && !(compliantExperiment && !compliantConverged)) {
                    const std::uint64_t ncpVictim = out.worstContactId;
                    for (std::size_t i = 0; i < contactIds.size(); ++i) {
                        if (contactIds[i] == ncpVictim) {
                            continue;
                        }
                        const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                        impl_->contactWarmStarts[contactIds[i]] = {
                            impulses.segment<3>(offset),
                            solvedContactVelocities.segment<3>(offset),
                            subDt,
                            ContactFrameRotation(pendingContacts[i].geometry.normal),
                            true};
                    }
                }
                out.failureReason = ProximalFailureReason::SolverNotConverged;
                return false;
            }
            if (!IsFinite(impulses)) {
                out.failureReason = ProximalFailureReason::NonFiniteImpulse;
                return false;
            }
            if (!IsFinite(solvedContactVelocities)) {
                out.failureReason = ProximalFailureReason::NonFiniteVelocity;
                return false;
            }
            std::unordered_set<std::uint64_t> activeIds;
            impl_->lastLegNormalImpulse.fill(0.0);
            impl_->haveLegNormalImpulseHistory = true;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                const Eigen::Vector3d impulse = impulses.segment<3>(
                    offset);
                const Eigen::Vector3d velocity = solvedContactVelocities.segment<3>(offset);
                impl_->contactWarmStarts[contactIds[i]] = {
                    impulse, velocity, subDt,
                    ContactFrameRotation(pendingContacts[i].geometry.normal),
                    true};
                activeIds.insert(contactIds[i]);
                out.peakNormalImpulse = std::max(out.peakNormalImpulse, std::abs(impulse[2]));
                out.peakFrictionImpulse = std::max(
                    out.peakFrictionImpulse, impulse.head<2>().norm());
                const Eigen::Vector3d worldFriction =
                    ContactFrameRotation(pendingContacts[i].geometry.normal)
                    * Eigen::Vector3d(impulse[0], impulse[1], 0.0);
                out.sumFrictionImpulseWorldX += worldFriction.x();
                out.sumFrictionImpulseWorldZ += worldFriction.z();
                out.sumAbsFrictionImpulseWorldX += std::abs(worldFriction.x());
                out.sumAbsFrictionImpulseWorldZ += std::abs(worldFriction.z());
                out.sumFrictionImpulseWorldY += worldFriction.y();
                if (pendingContacts[i].legIndex < 6) {
                    const std::size_t leg = pendingContacts[i].legIndex;
                    out.legFrictionImpulseWorldX[leg] += worldFriction.x();
                    out.legFrictionImpulseWorldZ[leg] += worldFriction.z();
                    impl_->lastLegNormalImpulse[leg] += std::max(0.0, impulse[2]);
                    if (out.legContactCount[leg] < 255) {
                        ++out.legContactCount[leg];
                    }
                }
            }
            bufferNormals.resize(contactIds.size());
            bufferImpulses.resize(contactIds.size());
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const Vec3& normal = pendingContacts[i].geometry.normal;
                bufferNormals[i] = {normal.x, normal.y, normal.z};
                const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                const Eigen::Vector3d storedImpulse = impulses.segment<3>(offset);
                bufferImpulses[i] = {storedImpulse.x(), storedImpulse.y(), storedImpulse.z()};
            }
            for (auto it = impl_->contactWarmStarts.begin(); it != impl_->contactWarmStarts.end();) {
                if (activeIds.find(it->first) == activeIds.end()) {
                    it = impl_->contactWarmStarts.erase(it);
                    ++impl_->totalWarmStartResets;
                } else {
                    ++it;
                }
            }
            // Apply the final ADMM iterate, rather than the articulated
            // Delassus operator's mutable scratch vector (which may contain
            // the most recent iterative product). A zero-velocity,
            // zero-gravity ABA evaluates M^-1 J^T lambda in linear time.
            Eigen::VectorXd generalizedImpulse = Eigen::VectorXd::Zero(impl_->model.nv);
            pinocchio::evalConstraintJacobianTransposeMatrixProduct(
                impl_->model,
                impl_->data,
                constraintModels,
                constraintDatas,
                impulses,
                generalizedImpulse,
                pinocchio::SetTo());
            if (!IsFinite(generalizedImpulse)) {
                out.failureReason = ProximalFailureReason::NonFiniteImpulse;
                return false;
            }
            Eigen::VectorXd contactDdq;
            if (haveImplicitH) {
                contactDdq = implicitSolve.Hldlt.solve(generalizedImpulse);
            } else {
                const Eigen::Vector3d savedGravity = impl_->model.gravity.linear();
                impl_->model.gravity.setZero();
                contactDdq = pinocchio::aba(
                    impl_->model,
                    impl_->data,
                    q,
                    Eigen::VectorXd::Zero(impl_->model.nv),
                    generalizedImpulse,
                    pinocchio::Convention::WORLD);
                impl_->model.gravity.linear() = savedGravity;
            }
            if (!IsFinite(contactDdq)) {
                out.failureReason = ProximalFailureReason::NonFiniteVelocity;
                return false;
            }
            vNew += contactDdq;
            const Eigen::Quaterniond chassisQuat(q[6], q[3], q[4], q[5]);
            const Eigen::Matrix3d bodyToWorld = chassisQuat.normalized().toRotationMatrix();
            const Eigen::Vector3d contactDeltaWorld = bodyToWorld * contactDdq.segment<3>(0);
            out.contactDeltaVx += contactDeltaWorld.x();
            out.contactDeltaVz += contactDeltaWorld.z();
        } else {
            impl_->contactConstraintModels.clear();
            impl_->contactConstraintDatas.clear();
            impl_->contactTopologyIds.clear();
            impl_->contactDelassus.reset();
            impl_->contactDelassusRegularization = 0.0;
            impl_->lastLegNormalImpulse.fill(0.0);
            impl_->haveLegNormalImpulseHistory = true;
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            out.contactSetupTimeMs += elapsedMs(contactSetupStart);
            if (!impl_->contactWarmStarts.empty() || impl_->haveLastContactSetSignature) {
                resetWarmStarts();
            }
        }

        const auto integrationStart = StepClock::now();
        if (!IsFinite(vNew)) {
            out.failureReason = ProximalFailureReason::NonFiniteVelocity;
            return false;
        }
        pinocchio::forwardKinematics(impl_->model, impl_->data, q, vNew);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);
        const std::uint32_t chassisBodyId = impl_->bodies.front().bodyId;
        const Impl::BodyBinding* speedLimitWinner = nullptr;
        double speedLimitWinnerSeverity = -1.0;
        double speedLimitWinnerLinear = -1.0;
        double speedLimitWinnerAngular = -1.0;
        bool speedLimitTripped = false;
        for (const Impl::BodyBinding& binding : impl_->bodies) {
            const pinocchio::Motion motion = pinocchio::getFrameVelocity(
                impl_->model, impl_->data, binding.frameId, pinocchio::LOCAL_WORLD_ALIGNED);
            const double linearSpeed = motion.linear().norm();
            const double angularSpeed = motion.angular().norm();
            out.preIntegrationLinearSpeed = std::max(out.preIntegrationLinearSpeed, linearSpeed);
            out.preIntegrationAngularSpeed = std::max(out.preIntegrationAngularSpeed, angularSpeed);
            if (binding.bodyId == chassisBodyId) {
                out.chassisPreIntegrationAngularSpeed = angularSpeed;
            } else {
                out.maxLinkPreIntegrationAngularSpeed =
                    std::max(out.maxLinkPreIntegrationAngularSpeed, angularSpeed);
            }
            const bool nonFinite = !std::isfinite(linearSpeed) || !std::isfinite(angularSpeed);
            const bool overLimit = linearSpeed > settings.maxLinearSpeed
                || angularSpeed > settings.maxAngularSpeed;
            if (!nonFinite && !overLimit) {
                continue;
            }
            speedLimitTripped = true;
            const double linearSeverity = std::isfinite(linearSpeed)
                ? linearSpeed / std::max(settings.maxLinearSpeed, 1.0e-12)
                : std::numeric_limits<double>::infinity();
            const double angularSeverity = std::isfinite(angularSpeed)
                ? angularSpeed / std::max(settings.maxAngularSpeed, 1.0e-12)
                : std::numeric_limits<double>::infinity();
            const double severity = std::max(linearSeverity, angularSeverity);
            if (speedLimitWinner == nullptr || severity >= speedLimitWinnerSeverity) {
                speedLimitWinner = &binding;
                speedLimitWinnerSeverity = severity;
                speedLimitWinnerLinear = linearSpeed;
                speedLimitWinnerAngular = angularSpeed;
            }
        }
        if (speedLimitTripped) {
            if (speedLimitWinner != nullptr) {
                const std::string& frameName =
                    impl_->model.frames[speedLimitWinner->frameId].name;
                out.speedLimitFrame = SpeedLimitFrameFromName(frameName);
                if (const std::optional<std::size_t> leg = SpeedLimitLegIndexFromName(frameName);
                    leg.has_value()) {
                    out.speedLimitLegIndex = static_cast<int>(*leg);
                    out.speedLimitSupport = loadBearingLegs[*leg]
                        ? ProximalSpeedLimitSupport::Stance
                        : ProximalSpeedLimitSupport::Swing;
                }
                if (traceSpeedLimit) {
                    std::cerr << "[proximal-speed-limit] frame=" << frameName
                              << " class=" << static_cast<unsigned>(out.speedLimitFrame)
                              << " support=" << static_cast<unsigned>(out.speedLimitSupport)
                              << " winner_v=" << speedLimitWinnerLinear
                              << " winner_w=" << speedLimitWinnerAngular
                              << " winner_severity=" << speedLimitWinnerSeverity
                              << " max_linear=" << out.preIntegrationLinearSpeed
                              << " chassis_w=" << out.chassisPreIntegrationAngularSpeed
                              << " max_link_w=" << out.maxLinkPreIntegrationAngularSpeed
                              << " pre_w=" << out.preIntegrationAngularSpeed
                              << " pd_gain=" << pdGainScale
                              << " damping_gain=" << (retryKeepDamping ? 1.0 : pdGainScale)
                              << " dt=" << subDt
                              << " peak_joint_err=" << out.peakPdAbsError
                              << '\n';
                    if (const std::optional<std::size_t> leg =
                            SpeedLimitLegIndexFromName(frameName);
                        leg.has_value()) {
                        std::cerr << "[proximal-speed-limit-joints] leg=" << *leg;
                        for (std::size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
                            const std::size_t wire = 3U * *leg + jointOffset;
                            const pinocchio::JointIndex joint = impl_->wireJoints[wire];
                            const Eigen::Index qi = impl_->model.joints[joint].idx_q();
                            const Eigen::Index vi = impl_->model.joints[joint].idx_v();
                            const double error = std::remainder(
                                activeServoTargets[wire] - impl_->wireZeroAngles[wire] - q[qi],
                                6.28318530717958647692);
                            std::cerr << " j" << jointOffset
                                      << "_vin=" << v[vi]
                                      << " j" << jointOffset << "_vaba=" << vBeforeContact[vi]
                                      << " j" << jointOffset << "_vnew=" << vNew[vi]
                                      << " j" << jointOffset << "_contact_dv="
                                      << (vNew[vi] - vBeforeContact[vi])
                                      << " j" << jointOffset << "_tau=" << tau[vi]
                                      << " j" << jointOffset << "_err=" << error;
                        }
                        std::cerr << '\n';
                    }
                }
                // One-shot initiating trip only (full PD). Retry gain 0.5 is
                // the leftover cascade, not the class we need to freeze.
                if (pdGainScale == 1.0) {
                    if (const char* snapshotPath =
                            std::getenv("HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH");
                        snapshotPath != nullptr && snapshotPath[0] != '\0'
                        && !impl_->speedLimitSnapshotCaptured) {
                        const auto safe = [](double value) {
                            return std::isfinite(value) ? value : 0.0;
                        };
                        const auto jsonString = [](const std::string& value) {
                            std::string out = "\"";
                            for (const char ch : value) {
                                if (ch == '"' || ch == '\\') {
                                    out.push_back('\\');
                                }
                                out.push_back(ch);
                            }
                            out.push_back('"');
                            return out;
                        };
                        try {
                            std::error_code existsError;
                            if (std::filesystem::exists(snapshotPath, existsError)) {
                                impl_->speedLimitSnapshotCaptured = true;
                                std::cerr << "[proximal-speed-limit-snapshot] skip existing path="
                                          << snapshotPath << '\n';
                            } else {
                            std::ofstream fixture(snapshotPath, std::ios::trunc);
                            if (!fixture) {
                                throw std::runtime_error("cannot open speed-limit snapshot path");
                            }
                            fixture << std::setprecision(17);
                            fixture << "{\"schema_version\":2,\"kind\":\"speed_limit\",";
                            fixture << "\"implicit_damping\":"
                                    << (impl_->implicitDamping ? "true" : "false");
                            fixture << ",\"pd_gain\":" << safe(pdGainScale)
                                    << ",\"dt\":" << safe(subDt)
                                    << ",\"command_interval_s\":" << safe(impl_->commandIntervalS)
                                    << ",\"no_load_speed\":" << safe(noLoadSpeed)
                                    << ",\"recovery_slew_active\":"
                                    << (recoverySlewActive ? "true" : "false")
                                    << ",\"max_linear_speed\":" << safe(settings.maxLinearSpeed)
                                    << ",\"max_angular_speed\":" << safe(settings.maxAngularSpeed)
                                    << ",\"winner\":{\"frame\":" << jsonString(frameName)
                                    << ",\"class\":" << static_cast<unsigned>(out.speedLimitFrame)
                                    << ",\"support\":" << static_cast<unsigned>(out.speedLimitSupport)
                                    << ",\"winner_v\":" << safe(speedLimitWinnerLinear)
                                    << ",\"winner_w\":" << safe(speedLimitWinnerAngular)
                                    << ",\"winner_severity\":" << safe(speedLimitWinnerSeverity)
                                    << "},\"chassis_w\":" << safe(out.chassisPreIntegrationAngularSpeed)
                                    << ",\"max_link_w\":" << safe(out.maxLinkPreIntegrationAngularSpeed)
                                    << ",\"pre_w\":" << safe(out.preIntegrationAngularSpeed)
                                    << ",\"max_linear\":" << safe(out.preIntegrationLinearSpeed)
                                    << ",\"peak_pd_abs_error\":" << safe(out.peakPdAbsError)
                                    << ",\"ncp\":{\"accepted\":true"
                                    << ",\"admm_converged\":"
                                    << (out.admmConverged ? "true" : "false")
                                    << ",\"ncp_physically_converged\":"
                                    << (out.ncpPhysicallyConverged ? "true" : "false")
                                    << ",\"iterations\":" << out.iterations
                                    << ",\"ncp_dual\":" << safe(out.ncpDualResidual)
                                    << ",\"ncp_complementarity\":"
                                    << safe(out.ncpComplementarityResidual)
                                    << ",\"cone\":" << safe(out.coneResidual)
                                    << ",\"compliant_projected_residual\":"
                                    << safe(out.compliantProjectedResidual)
                                    << ",\"contact_constraint_count\":"
                                    << out.contactConstraintCount
                                    << ",\"peak_normal_impulse\":"
                                    << safe(out.peakNormalImpulse)
                                    << ",\"max_contact_penetration\":"
                                    << safe(out.maxContactPenetration)
                                    << "},";
                            const std::optional<std::size_t> winnerLeg =
                                SpeedLimitLegIndexFromName(frameName);
                            const double commandDt = impl_->commandIntervalS > 1.0e-9
                                ? impl_->commandIntervalS
                                : subDt;
                            if (winnerLeg.has_value()) {
                                double composedVinAbsSum = 0.0;
                                fixture << "\"winner_leg\":" << *winnerLeg << ",\"joints\":[";
                                for (std::size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
                                    const std::size_t wire = 3U * *winnerLeg + jointOffset;
                                    const pinocchio::JointIndex joint = impl_->wireJoints[wire];
                                    const Eigen::Index qi = impl_->model.joints[joint].idx_q();
                                    const Eigen::Index vi = impl_->model.joints[joint].idx_v();
                                    const double error = std::remainder(
                                        activeServoTargets[wire] - impl_->wireZeroAngles[wire] - q[qi],
                                        6.28318530717958647692);
                                    const double targetDelta = std::remainder(
                                        activeServoTargets[wire] - previousCommanded[wire],
                                        6.28318530717958647692);
                                    const double servoTargetVelocity =
                                        world.GetServoJoint(impl_->wireServoIds[wire]).targetVelocity;
                                    composedVinAbsSum += std::abs(v[vi]);
                                    const bool saturated =
                                        std::abs(requestedTau[wire])
                                        > std::abs(availableTau[wire]) + 1.0e-12;
                                    if (jointOffset != 0) {
                                        fixture << ',';
                                    }
                                    fixture << "{\"offset\":" << jointOffset
                                            << ",\"wire\":" << wire
                                            << ",\"vin\":" << safe(v[vi])
                                            << ",\"vaba\":" << safe(vBeforeContact[vi])
                                            << ",\"vnew\":" << safe(vNew[vi])
                                            << ",\"contact_dv\":"
                                            << safe(vNew[vi] - vBeforeContact[vi])
                                            << ",\"tau\":" << safe(tau[vi])
                                            << ",\"requested_tau\":" << safe(requestedTau[wire])
                                            << ",\"available_tau\":" << safe(availableTau[wire])
                                            << ",\"tau_saturated\":"
                                            << (saturated ? "true" : "false")
                                            << ",\"err\":" << safe(error)
                                            << ",\"target_angle\":"
                                            << safe(activeServoTargets[wire])
                                            << ",\"raw_server_target\":" << safe(servoTargets[wire])
                                            << ",\"target_rate_radps\":" << safe(servoTargetVelocity)
                                            << ",\"commanded_rate_radps\":"
                                            << safe(targetDelta / commandDt)
                                            << "}";
                                }
                                fixture << "],\"composed_vin_abs_sum\":"
                                        << safe(composedVinAbsSum);
                            } else {
                                fixture << "\"winner_leg\":null,\"joints\":[],"
                                           "\"composed_vin_abs_sum\":0";
                            }
                            // Diagnostic-only oracle, after rejection. Separate Data
                            // keeps the production solver scratch/warm starts untouched.
                            // Preserve LOCAL tangent coordinates even though production
                            // ABA uses WORLD for its internal articulated recursion.
                            pinocchio::Data auditData(impl_->model);
                            Eigen::MatrixXd auditMass = pinocchio::crba(impl_->model, auditData, q);
                            auditMass.triangularView<Eigen::StrictlyLower>() =
                                auditMass.transpose().triangularView<Eigen::StrictlyLower>();
                            const Eigen::VectorXd auditBias = pinocchio::nonLinearEffects(impl_->model, auditData, q, v);
                            const Eigen::VectorXd auditGravity = pinocchio::rnea(impl_->model, auditData, q,
                                Eigen::VectorXd::Zero(impl_->model.nv), Eigen::VectorXd::Zero(impl_->model.nv));
                            const auto writeAuditVector = [&](const Eigen::VectorXd& vector) {
                                fixture << '[';
                                for (Eigen::Index i = 0; i < vector.size(); ++i) {
                                    if (i != 0) fixture << ',';
                                    fixture << vector[i];
                                }
                                fixture << ']';
                            };
                            const auto writeAuditMatrix = [&](const Eigen::MatrixXd& matrix) {
                                fixture << '[';
                                for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
                                    if (row != 0) fixture << ',';
                                    writeAuditVector(matrix.row(row).transpose());
                                }
                                fixture << ']';
                            };
                            fixture << ",\"kinematics\":{\"tangent_convention\":\"free_flyer_local_linear_then_angular\""
                                    << ",\"angular_frame\":\"LOCAL_WORLD_ALIGNED\",\"q\":";
                            writeAuditVector(q);
                            fixture << ",\"v_in\":"; writeAuditVector(v);
                            fixture << ",\"v_free\":"; writeAuditVector(vBeforeContact);
                            fixture << ",\"v_after\":"; writeAuditVector(vNew);
                            fixture << ",\"tau\":"; writeAuditVector(tau);
                            fixture << ",\"mass\":"; writeAuditMatrix(auditMass);
                            fixture << ",\"nonlinear_force\":"; writeAuditVector(auditBias);
                            fixture << ",\"gravity_force\":"; writeAuditVector(auditGravity);
                            fixture << ",\"omega_n\":" << omegaN << ",\"zeta\":" << zeta
                                    << ",\"servo_gain_scale\":" << impl_->servoGainScale
                                    << ",\"wires\":[";
                            for (std::size_t wire = 0; wire < impl_->wireJoints.size(); ++wire) {
                                if (wire != 0) fixture << ',';
                                const auto joint = impl_->wireJoints[wire];
                                const auto qi = impl_->model.joints[joint].idx_q();
                                const auto vi = impl_->model.joints[joint].idx_v();
                                fixture << "{\"wire\":" << wire << ",\"q_index\":" << qi
                                        << ",\"v_index\":" << vi << ",\"zero_angle\":" << impl_->wireZeroAngles[wire]
                                        << ",\"error\":" << std::remainder(activeServoTargets[wire] - impl_->wireZeroAngles[wire] - q[qi], 6.28318530717958647692)
                                        << ",\"target_rate\":" << world.GetServoJoint(impl_->wireServoIds[wire]).targetVelocity
                                        << ",\"effective_inertia\":" << impl_->wireEffectiveInertias[wire]
                                        << ",\"requested_tau\":" << requestedTau[wire]
                                        << ",\"available_tau\":" << availableTau[wire] << '}';
                            }
                            fixture << "],\"links\":[";
                            pinocchio::computeJointJacobians(impl_->model, auditData, q);
                            pinocchio::updateFramePlacements(impl_->model, auditData);
                            bool firstLink = true;
                            for (const auto& binding : impl_->bodies) {
                                if (!firstLink) fixture << ',';
                                firstLink = false;
                                Eigen::Matrix<double, 6, Eigen::Dynamic> frameJacobian(6, impl_->model.nv);
                                frameJacobian.setZero();
                                pinocchio::getFrameJacobian(impl_->model, auditData, binding.frameId,
                                    pinocchio::LOCAL_WORLD_ALIGNED, frameJacobian);
                                const Eigen::MatrixXd angularJacobian = frameJacobian.bottomRows<3>();
                                fixture << "{\"name\":" << jsonString(impl_->model.frames[binding.frameId].name)
                                        << ",\"body_id\":" << binding.bodyId << ",\"angular_jacobian\":";
                                writeAuditMatrix(angularJacobian);
                                fixture << ",\"angular_in\":"; writeAuditVector(angularJacobian * v);
                                fixture << ",\"angular_free\":"; writeAuditVector(angularJacobian * vBeforeContact);
                                fixture << ",\"angular_after\":"; writeAuditVector(angularJacobian * vNew);
                                fixture << '}';
                            }
                            fixture << ']';
                            fixture << '}';
                            fixture << ',';
                            writeAcceptedHistoryJson(fixture);
                            fixture << "}\n";
                            fixture.close();
                            impl_->speedLimitSnapshotCaptured = true;
                            dumpPrefailureBuffer("speed_limit");
                            std::cerr << "[proximal-speed-limit-snapshot] path="
                                      << snapshotPath
                                      << " frame=" << frameName
                                      << " winner_w=" << speedLimitWinnerAngular
                                      << '\n';
                            }
                        } catch (const std::exception& ex) {
                            std::cerr << "[proximal-speed-limit-snapshot] failed: "
                                      << ex.what() << '\n';
                        }
                    }
                }
            }
            out.failureReason = ProximalFailureReason::SpeedLimit;
            if (pdGainScale == 1.0) {
                dumpPrefailureBuffer("speed_limit");
            }
            return false;
        }

        const Eigen::VectorXd qNew = pinocchio::integrate(impl_->model, q, subDt * vNew);
        if (!IsFinite(qNew)) {
            out.failureReason = ProximalFailureReason::NonFiniteConfiguration;
            return false;
        }
        std::vector<double> qNewStorage(qNew.data(), qNew.data() + qNew.size());
        std::vector<double> vNewStorage(vNew.data(), vNew.data() + vNew.size());
        if (!writeValidatedState(world, qNewStorage, vNewStorage, settings, out)) {
            if (out.failureReason == ProximalFailureReason::SpeedLimit) {
                if (out.speedLimitLegIndex >= 0) {
                    out.speedLimitSupport = loadBearingLegs[static_cast<std::size_t>(out.speedLimitLegIndex)]
                        ? ProximalSpeedLimitSupport::Stance : ProximalSpeedLimitSupport::Swing;
                }
                // The old-pose guard passed. Pose transport can still amplify
                // the same tangent velocity, so reject before writing bodies.
                if (traceSpeedLimit) {
                    std::cerr << "[proximal-integrated-speed-limit] class="
                              << static_cast<unsigned>(out.speedLimitFrame)
                              << " leg=" << out.speedLimitLegIndex
                              << " max_linear=" << out.preIntegrationLinearSpeed
                              << " max_angular=" << out.preIntegrationAngularSpeed
                              << " pd_gain=" << pdGainScale << " dt=" << subDt << '\n';
                }
                if (pdGainScale == 1.0) {
                    dumpPrefailureBuffer("speed_limit");
                }
            }
            return false;
        }
        {
            Impl::AcceptedStateBufferEntry sample;
            sample.q = qNewStorage;
            sample.v = vNewStorage;
            const Body& chassis = world.GetBody(impl_->bodies.front().bodyId);
            sample.chassisAngularWorld = {
                chassis.angularVelocity.x,
                chassis.angularVelocity.y,
                chassis.angularVelocity.z};
            for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
                const pinocchio::JointIndex joint = impl_->wireJoints[i];
                const Eigen::Index qi = impl_->model.joints[joint].idx_q();
                const Eigen::Index vi = impl_->model.joints[joint].idx_v();
                sample.tau[i] = tau[vi];
                sample.targets[i] = activeServoTargets[i];
                sample.errors[i] = std::remainder(
                    activeServoTargets[i] - impl_->wireZeroAngles[i] - q[qi],
                    6.28318530717958647692);
                sample.effectiveInertias[i] = impl_->wireEffectiveInertias[i];
            }
            sample.subDt = subDt;
            sample.servoGainScale = impl_->servoGainScale * loadGainScale * pdGainScale;
            sample.servoDampingGainScale = impl_->servoGainScale * loadGainScale
                * (retryKeepDamping ? 1.0 : pdGainScale);
            sample.commandDt = impl_->commandIntervalS;
            sample.ncpCcpRecovery = out.ncpCcpRecovery;
            sample.warmStartCount = impl_->contactWarmStarts.size();
            sample.loadBearingMask = impl_->lastLoadBearingMask;
            sample.reducedSupportBlend = impl_->staticReducedSupportGainBlend;
            sample.contactIds = contactIds;
            sample.normals = bufferNormals;
            sample.impulses = bufferImpulses;
            impl_->acceptedStateBuffer.push_back(std::move(sample));
            while (impl_->acceptedStateBuffer.size() > Impl::kAcceptedStateBufferCapacity) {
                impl_->acceptedStateBuffer.pop_front();
            }
        }
        const double energyAfter = totalMechanicalEnergy();
        const double energyDelta = energyAfter - energyBefore;
        if (!std::isfinite(energyDelta)) {
            out.failureReason = ProximalFailureReason::NonFiniteEnergy;
            return false;
        }
        out.mechanicalEnergyDelta += energyDelta;
        out.integrationTimeMs += elapsedMs(integrationStart);
        return true;
    };

    const double reducedSupportDwellSnapshot = impl_->staticReducedSupportDwellS;
    const double reducedSupportBlendSnapshot = impl_->staticReducedSupportGainBlend;
    const auto effectiveInertiasSnapshot = impl_->wireEffectiveInertias;
    const auto stiffnessSnapshot = impl_->servoStiffnessNmPerRad;
    const auto targetInertiasSnapshot = impl_->wireTargetInertias;
    const auto lastLoadBearingMaskSnapshot = impl_->lastLoadBearingMask;
    const auto lastLoadBearingCountSnapshot = impl_->lastLoadBearingCount;
    const bool haveLoadBearingMaskSnapshot = impl_->haveLoadBearingMask;
    const auto lastLegNormalImpulseSnapshot = impl_->lastLegNormalImpulse;
    const bool haveLegNormalImpulseHistorySnapshot = impl_->haveLegNormalImpulseHistory;
    const auto fullSupportTargetsSnapshot = impl_->staticFullSupportServoTargets;
    const bool haveFullSupportTargetsSnapshot = impl_->haveStaticFullSupportServoTargets;
    const auto preStepWarmStarts = impl_->contactWarmStarts;
    const auto restoreReducedSupportController = [&]() {
        impl_->staticReducedSupportDwellS = reducedSupportDwellSnapshot;
        impl_->staticReducedSupportGainBlend = reducedSupportBlendSnapshot;
        impl_->wireEffectiveInertias = effectiveInertiasSnapshot;
        impl_->servoStiffnessNmPerRad = stiffnessSnapshot;
        impl_->wireTargetInertias = targetInertiasSnapshot;
        impl_->lastLoadBearingMask = lastLoadBearingMaskSnapshot;
        impl_->lastLoadBearingCount = lastLoadBearingCountSnapshot;
        impl_->haveLoadBearingMask = haveLoadBearingMaskSnapshot;
        impl_->lastLegNormalImpulse = lastLegNormalImpulseSnapshot;
        impl_->haveLegNormalImpulseHistory = haveLegNormalImpulseHistorySnapshot;
        impl_->staticFullSupportServoTargets = fullSupportTargetsSnapshot;
        impl_->haveStaticFullSupportServoTargets = haveFullSupportTargetsSnapshot;
        contactInertiaRetargetedThisStep = false;
    };
    ProximalStepDiagnostics firstAttempt{};
    const auto clearFailedSolverState = [&](const std::uint64_t worstContactId) {
        if (worstContactId == 0) {
            resetWarmStarts();
            return;
        }
        impl_->totalWarmStartResets +=
            impl_->contactWarmStarts.erase(worstContactId);
        impl_->contactSolverResult.reset();
        impl_->contactSolver.reset();
    };
    if (advanceOnce(dt, commandedServoTargets, impl_->andersonCapacity, firstAttempt)) {
        world.CompleteExternalDynamicsStep();
        impl_->commandedServoTargets = commandedServoTargets;
        impl_->haveCommandedServoTargets = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::Healthy;
        readState(world, impl_->lastGoodQ, impl_->lastGoodV);
    } else if (unsupportedIsland) {
        ++impl_->totalUnsupportedIslands;
        ++impl_->totalRollbacks;
        writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        restoreReducedSupportController();
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        impl_->servoTargetRecoveryActive = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::UnsupportedIsland;
    } else if (firstAttempt.failureReason != ProximalFailureReason::SolverNotConverged
               && firstAttempt.failureReason != ProximalFailureReason::SpeedLimit) {
        // Retrying with two half-steps can only help failures that depend on
        // integration size or iterative convergence. Invalid/non-finite state,
        // excessive penetration, and state write failures are unchanged by a
        // smaller dt, so publish a held last-good sample without performing two
        // redundant collision and contact solves.
        ++impl_->totalRollbacks;
        ++impl_->totalHeldStates;
        writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        restoreReducedSupportController();
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::HeldLastGood;
    } else {
        ++impl_->totalRetries;
        ++impl_->totalRollbacks;
        if (traceFailures) {
            std::cerr << "[proximal-first-failure] reason="
                      << static_cast<unsigned>(firstAttempt.failureReason)
                      << " iterations=" << firstAttempt.iterations
                      << " primal=" << firstAttempt.primalResidual
                      << " dual=" << firstAttempt.dualResidual
                      << " comp=" << firstAttempt.complementarityResidual
                      << " ncp_dual=" << firstAttempt.ncpDualResidual
                      << " ncp_comp=" << firstAttempt.ncpComplementarityResidual
                      << " cone=" << firstAttempt.coneResidual
                      << " contacts=" << firstAttempt.contactConstraintCount
                      << " signature=" << firstAttempt.contactSetSignature
                      << " worst_contact=" << firstAttempt.worstContactId
                      << " rho=" << firstAttempt.admmRho
                      << " pre_v=" << firstAttempt.preIntegrationLinearSpeed
                      << " pre_w=" << firstAttempt.preIntegrationAngularSpeed
                      << " chassis_w=" << firstAttempt.chassisPreIntegrationAngularSpeed
                      << " max_link_w=" << firstAttempt.maxLinkPreIntegrationAngularSpeed
                      << " speed_frame=" << static_cast<unsigned>(firstAttempt.speedLimitFrame)
                      << " speed_support=" << static_cast<unsigned>(firstAttempt.speedLimitSupport)
                      << " pd_gain=1"
                      << " dt=" << dt
                      << " peak_joint_err=" << firstAttempt.peakPdAbsError
                      << '\n';
        }
        const auto preRetryWarmStarts = impl_->contactWarmStarts;
        writeState(world, snapshotQ, snapshotV);
        restoreReducedSupportController();
        clearFailedSolverState(firstAttempt.worstContactId);
        ProximalStepDiagnostics retry{};
        const auto accumulateRetryTimings = [&]() {
            retry.dynamicsTimeMs += firstAttempt.dynamicsTimeMs;
            retry.contactSetupTimeMs += firstAttempt.contactSetupTimeMs;
            retry.collisionTimeMs += firstAttempt.collisionTimeMs;
            retry.constraintAssemblyTimeMs += firstAttempt.constraintAssemblyTimeMs;
            retry.delassusTimeMs += firstAttempt.delassusTimeMs;
            retry.admmTimeMs += firstAttempt.admmTimeMs;
            retry.integrationTimeMs += firstAttempt.integrationTimeMs;
        };
        // Keep the better-converged primary history length for retries. A
        // shorter adaptive history increased held states on both the original
        // frozen stream and a genuine, non-inhibited gait stream.
        const std::size_t retryAndersonCapacity =
            impl_->retryAndersonCapacityOverride.value_or(impl_->andersonCapacity);
        const double retryPdGain =
            firstAttempt.failureReason == ProximalFailureReason::SpeedLimit
                ? kSpeedLimitRetryGainScale
                : 1.0;
        bool recovered = false;
        if ((firstAttempt.failureReason == ProximalFailureReason::SolverNotConverged
             || firstAttempt.failureReason == ProximalFailureReason::SpeedLimit)
            && advanceOnce(
                dt,
                commandedServoTargets,
                retryAndersonCapacity,
                retry,
                0,
                retryPdGain)) {
            recovered = true;
        } else {
            writeState(world, snapshotQ, snapshotV);
            restoreReducedSupportController();
            impl_->contactWarmStarts = preRetryWarmStarts;
            clearFailedSolverState(
                retry.worstContactId != 0
                    ? retry.worstContactId
                    : firstAttempt.worstContactId);
            const bool half1 = advanceOnce(
                0.5 * dt,
                commandedServoTargets,
                retryAndersonCapacity,
                retry,
                0,
                retryPdGain);
            recovered = half1
                && advanceOnce(
                    0.5 * dt,
                    commandedServoTargets,
                    retryAndersonCapacity,
                    retry,
                    0,
                    retryPdGain);
            if (!recovered) {
                // Last resort after same-dt + warm half-steps: cold-start two
                // half-steps at 2× SolverIterations. Poisoned 24-iter warms can
                // keep a 5-contact set above the 1e-3 NCP floor even at dt/2.
                // Healthy-path first attempts stay at the production cap.
                const ProximalStepDiagnostics failedHalf = retry;
                writeState(world, snapshotQ, snapshotV);
                restoreReducedSupportController();
                resetWarmStarts();
                const int coldIterations = std::max(2 * settings.maxIterations, 48);
                // Last-resort-only: census cold 48 often lands just above the
                // 1e-3 NCP floor (e.g. 0.00157). Do not loosen first-attempt
                // or standing ADMM (1e-8). SpeedLimit first-fails keep the
                // production NCP floor so this does not mask leftover-ω.
                const double coldNcpAccept =
                    firstAttempt.failureReason
                            == ProximalFailureReason::SolverNotConverged
                        ? 2.0 * settings.ncpAbsoluteTolerance
                        : 0.0;
                ProximalStepDiagnostics cold{};
                // Last-resort stays two cold half-steps. Four dt/4 NCP steps
                // recovered some 5-contact snapshots under 2×, then dumped
                // leftover-ω tibia SpeedLimit on forward (streak 334) and
                // broke isolated straight stay-WALK. Keep all contacts.
                const bool cold1 = advanceOnce(
                    0.5 * dt,
                    commandedServoTargets,
                    retryAndersonCapacity,
                    cold,
                    coldIterations,
                    retryPdGain,
                    coldNcpAccept,
                    0,
                    0.0);
                recovered = cold1
                    && advanceOnce(
                        0.5 * dt,
                        commandedServoTargets,
                        retryAndersonCapacity,
                        cold,
                        coldIterations,
                        retryPdGain,
                        coldNcpAccept,
                        0,
                        0.0);
                cold.dynamicsTimeMs += failedHalf.dynamicsTimeMs;
                cold.contactSetupTimeMs += failedHalf.contactSetupTimeMs;
                cold.collisionTimeMs += failedHalf.collisionTimeMs;
                cold.constraintAssemblyTimeMs += failedHalf.constraintAssemblyTimeMs;
                cold.delassusTimeMs += failedHalf.delassusTimeMs;
                cold.admmTimeMs += failedHalf.admmTimeMs;
                cold.integrationTimeMs += failedHalf.integrationTimeMs;
                retry = cold;
                if (!recovered
                    && !disableNcpCcpRecovery
                    && !compliantContactSession
                    && firstAttempt.failureReason
                        == ProximalFailureReason::SolverNotConverged) {
                    // Rigid last-resort still missed the Signorini NCP floor.
                    // Solve the well-posed Coulomb cone QP (same law as Mode 2)
                    // for two cold half-steps so the write is a real integrated
                    // state. Speed/impulse/projected-residual gates still apply.
                    writeState(world, snapshotQ, snapshotV);
                    restoreReducedSupportController();
                    resetWarmStarts();
                    ProximalStepDiagnostics ccp{};
                    const bool ccp1 = advanceOnce(
                        0.5 * dt,
                        commandedServoTargets,
                        retryAndersonCapacity,
                        ccp,
                        coldIterations,
                        retryPdGain,
                        0.0,
                        0,
                        0.0,
                        true);
                    recovered = ccp1
                        && advanceOnce(
                            0.5 * dt,
                            commandedServoTargets,
                            retryAndersonCapacity,
                            ccp,
                            coldIterations,
                            retryPdGain,
                            0.0,
                            0,
                            0.0,
                            true);
                    ccp.ncpCcpRecovery = true;
                    ccp.dynamicsTimeMs += cold.dynamicsTimeMs;
                    ccp.contactSetupTimeMs += cold.contactSetupTimeMs;
                    ccp.collisionTimeMs += cold.collisionTimeMs;
                    ccp.constraintAssemblyTimeMs += cold.constraintAssemblyTimeMs;
                    ccp.delassusTimeMs += cold.delassusTimeMs;
                    ccp.admmTimeMs += cold.admmTimeMs;
                    ccp.integrationTimeMs += cold.integrationTimeMs;
                    std::cerr << "[proximal-ncp-ccp-recovery]"
                              << " accept=" << (recovered ? 1 : 0)
                              << " contacts=" << ccp.contactConstraintCount
                              << " projected=" << ccp.compliantProjectedResidual
                              << " ncp_dual=" << ccp.ncpDualResidual
                              << " ncp_comp=" << ccp.ncpComplementarityResidual
                              << " peak_n=" << ccp.peakNormalImpulse
                              << " max_link_w=" << ccp.maxLinkPreIntegrationAngularSpeed
                              << " reason=" << static_cast<unsigned>(ccp.failureReason)
                              << '\n';
                    if (recovered) {
                        retry = ccp;
                    } else {
                        retry.ncpCcpRecovery = true;
                        writeState(world, snapshotQ, snapshotV);
                        restoreReducedSupportController();
                        resetWarmStarts();
                    }
                }
                if (!recovered
                    && traceFailures
                    && firstAttempt.failureReason
                        == ProximalFailureReason::SolverNotConverged) {
                    const std::uint64_t omitComp =
                        cold.worstComplementarityContactId != 0
                            ? cold.worstComplementarityContactId
                            : firstAttempt.worstComplementarityContactId;
                    const std::uint64_t omitDual =
                        cold.worstContactId != 0
                            ? cold.worstContactId
                            : firstAttempt.worstContactId;
                    const auto probeOmit = [&](const char* label,
                                               const std::uint64_t omitId) {
                        writeState(world, snapshotQ, snapshotV);
                        restoreReducedSupportController();
                        resetWarmStarts();
                        ProximalStepDiagnostics probe{};
                        const bool accepted = omitId != 0
                            && advanceOnce(
                                0.5 * dt,
                                commandedServoTargets,
                                retryAndersonCapacity,
                                probe,
                                coldIterations,
                                retryPdGain,
                                coldNcpAccept,
                                omitId);
                        std::cerr << "[proximal-held-omit] " << label
                                  << " omit=" << omitId
                                  << " accept=" << (accepted ? 1 : 0)
                                  << " ncp_dual=" << probe.ncpDualResidual
                                  << " ncp_comp=" << probe.ncpComplementarityResidual
                                  << " contacts=" << probe.contactConstraintCount
                                  << " unique_joints=" << probe.uniqueRobotJointCount
                                  << '\n';
                        writeState(world, snapshotQ, snapshotV);
                        restoreReducedSupportController();
                        resetWarmStarts();
                    };
                    probeOmit("comp", omitComp);
                    probeOmit("dual", omitDual);
                    writeState(world, snapshotQ, snapshotV);
                    restoreReducedSupportController();
                    resetWarmStarts();
                    ProximalStepDiagnostics quarter{};
                    const bool quarterAccepted = advanceOnce(
                        0.25 * dt,
                        commandedServoTargets,
                        retryAndersonCapacity,
                        quarter,
                        coldIterations,
                        retryPdGain,
                        coldNcpAccept);
                    std::cerr << "[proximal-held-quarter]"
                              << " accept=" << (quarterAccepted ? 1 : 0)
                              << " ncp_dual=" << quarter.ncpDualResidual
                              << " ncp_comp=" << quarter.ncpComplementarityResidual
                              << " contacts=" << quarter.contactConstraintCount
                              << " unique_joints=" << quarter.uniqueRobotJointCount
                              << '\n';
                    writeState(world, snapshotQ, snapshotV);
                    restoreReducedSupportController();
                    resetWarmStarts();
                    ProximalStepDiagnostics extraIters{};
                    const int extraIterations =
                        std::max(4 * settings.maxIterations, 96);
                    const bool extraAccepted = advanceOnce(
                        0.5 * dt,
                        commandedServoTargets,
                        retryAndersonCapacity,
                        extraIters,
                        extraIterations,
                        retryPdGain,
                        coldNcpAccept);
                    std::cerr << "[proximal-held-iters]"
                              << " accept=" << (extraAccepted ? 1 : 0)
                              << " ncp_dual=" << extraIters.ncpDualResidual
                              << " ncp_comp=" << extraIters.ncpComplementarityResidual
                              << " contacts=" << extraIters.contactConstraintCount
                              << " unique_joints=" << extraIters.uniqueRobotJointCount
                              << " iterations=" << extraIters.iterations
                              << '\n';
                    writeState(world, snapshotQ, snapshotV);
                    restoreReducedSupportController();
                    resetWarmStarts();
                    const auto probeGraze = [&](const double minPen) {
                        writeState(world, snapshotQ, snapshotV);
                        restoreReducedSupportController();
                        resetWarmStarts();
                        ProximalStepDiagnostics probe{};
                        const bool accepted = advanceOnce(
                            0.5 * dt,
                            commandedServoTargets,
                            retryAndersonCapacity,
                            probe,
                            coldIterations,
                            retryPdGain,
                            coldNcpAccept,
                            0,
                            minPen);
                        int dropped = 0;
                        bool kept13 = false;
                        for (std::size_t i = 0; i < probe.tracedContactCount; ++i) {
                            if (probe.tracedContactJoint[i] == 13) {
                                kept13 = true;
                            }
                        }
                        std::cerr << "[proximal-held-graze]"
                                  << " min_pen=" << minPen
                                  << " accept=" << (accepted ? 1 : 0)
                                  << " ncp_dual=" << probe.ncpDualResidual
                                  << " ncp_comp=" << probe.ncpComplementarityResidual
                                  << " contacts=" << probe.contactConstraintCount
                                  << " unique_joints=" << probe.uniqueRobotJointCount
                                  << " kept13=" << (kept13 ? 1 : 0)
                                  << " dropped=";
                        for (std::size_t i = 0; i < retry.tracedContactCount; ++i) {
                            if (retry.tracedContactPen[i] >= minPen) {
                                continue;
                            }
                            if (dropped != 0) {
                                std::cerr << ',';
                            }
                            std::cerr << retry.tracedContactId[i] << ':'
                                      << static_cast<unsigned>(retry.tracedContactLeg[i])
                                      << ':' << retry.tracedContactPen[i] << ':'
                                      << retry.tracedContactDual[i] << ':'
                                      << retry.tracedContactComp[i];
                            ++dropped;
                        }
                        std::cerr << " dropped_n=" << dropped << '\n';
                        writeState(world, snapshotQ, snapshotV);
                        restoreReducedSupportController();
                        resetWarmStarts();
                    };
                    probeGraze(5.0e-5);
                    probeGraze(1.0e-4);
                }
            }
        }
        accumulateRetryTimings();
        if (recovered) {
            world.CompleteExternalDynamicsStep();
            impl_->commandedServoTargets = commandedServoTargets;
            impl_->haveCommandedServoTargets = true;
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::RecoveredRetry;
            // Retain the reason that made recovery necessary. Consumers can
            // now distinguish a clean healthy step from a usable step that
            // recovered solver non-convergence or a speed-limit rejection.
            diagnostics.failureReason = firstAttempt.failureReason;
            if (firstAttempt.failureReason == ProximalFailureReason::SpeedLimit) {
                diagnostics.speedLimitFrame = firstAttempt.speedLimitFrame;
                diagnostics.speedLimitSupport = firstAttempt.speedLimitSupport;
            }
            diagnostics.chassisPreIntegrationAngularSpeed = std::max(
                diagnostics.chassisPreIntegrationAngularSpeed,
                firstAttempt.chassisPreIntegrationAngularSpeed);
            diagnostics.maxLinkPreIntegrationAngularSpeed = std::max(
                diagnostics.maxLinkPreIntegrationAngularSpeed,
                firstAttempt.maxLinkPreIntegrationAngularSpeed);
            readState(world, impl_->lastGoodQ, impl_->lastGoodV);
        } else {
            ++impl_->totalHeldStates;
            ++impl_->totalRollbacks;
            writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
            restoreReducedSupportController();
            {
                const std::size_t discarded = impl_->contactWarmStarts.size();
                impl_->contactWarmStarts = preStepWarmStarts;
                impl_->contactSolverResult.reset();
                impl_->contactSolver.reset();
                static std::uint64_t warmStartRestoreLogs = 0;
                if ((discarded != preStepWarmStarts.size()
                     || firstAttempt.failureReason == ProximalFailureReason::SpeedLimit
                     || firstAttempt.failureReason == ProximalFailureReason::SolverNotConverged)
                    && warmStartRestoreLogs < 8) {
                    ++warmStartRestoreLogs;
                    std::cerr << "[proximal-warm-start-restore] restored="
                              << preStepWarmStarts.size()
                              << " discarded=" << discarded << '\n';
                }
            }
            dumpPrefailureBuffer(
                firstAttempt.failureReason == ProximalFailureReason::SolverNotConverged
                    ? "ncp_hold"
                    : "held_last_good");
            for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
                impl_->commandedServoTargets[i] =
                    world.GetServoJointAngle(impl_->wireServoIds[i]);
            }
            impl_->haveCommandedServoTargets = true;
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::HeldLastGood;
            diagnostics.failureReason = firstAttempt.failureReason;
            if (firstAttempt.failureReason == ProximalFailureReason::SpeedLimit) {
                diagnostics.speedLimitFrame = firstAttempt.speedLimitFrame;
                diagnostics.speedLimitSupport = firstAttempt.speedLimitSupport;
            }
            diagnostics.chassisPreIntegrationAngularSpeed = std::max(
                diagnostics.chassisPreIntegrationAngularSpeed,
                firstAttempt.chassisPreIntegrationAngularSpeed);
            diagnostics.maxLinkPreIntegrationAngularSpeed = std::max(
                diagnostics.maxLinkPreIntegrationAngularSpeed,
                firstAttempt.maxLinkPreIntegrationAngularSpeed);
            if (traceFailures) {
                const auto printHeldContacts =
                    [](const char* label, const ProximalStepDiagnostics& sample) {
                        std::cerr << "[proximal-held-contacts] " << label
                                  << " duplicates=" << sample.duplicateContactCount
                                  << " unique_joints=" << sample.uniqueRobotJointCount
                                  << " worst_id=" << sample.worstContactId
                                  << " worst_comp_id="
                                  << sample.worstComplementarityContactId
                                  << " contacts=";
                        for (std::size_t i = 0; i < sample.tracedContactCount; ++i) {
                            std::cerr << (i == 0 ? "[" : ",")
                                      << sample.tracedContactId[i] << ':'
                                      << static_cast<unsigned>(sample.tracedContactLeg[i])
                                      << ':' << sample.tracedContactJoint[i] << ':'
                                      << sample.tracedContactComp[i] << ':'
                                      << sample.tracedContactDual[i] << ':'
                                      << sample.tracedContactCone[i] << ':'
                                      << sample.tracedContactPen[i];
                        }
                        std::cerr << "]\n";
                    };
                std::cerr << "[proximal-held]"
                          << " first_reason="
                          << static_cast<unsigned>(firstAttempt.failureReason)
                          << " first_iters=" << firstAttempt.iterations
                          << " first_ncp_dual=" << firstAttempt.ncpDualResidual
                          << " first_ncp_comp=" << firstAttempt.ncpComplementarityResidual
                          << " first_contacts=" << firstAttempt.contactConstraintCount
                          << " first_worst=" << firstAttempt.worstContactId
                          << " first_rho=" << firstAttempt.admmRho
                          << " first_peak_joint_err=" << firstAttempt.peakPdAbsError
                          << " retry_reason="
                          << static_cast<unsigned>(retry.failureReason)
                          << " retry_iters=" << retry.iterations
                          << " retry_ncp_dual=" << retry.ncpDualResidual
                          << " retry_ncp_comp=" << retry.ncpComplementarityResidual
                          << " retry_contacts=" << retry.contactConstraintCount
                          << " retry_worst=" << retry.worstContactId
                          << " retry_rho=" << retry.admmRho
                          << " retry_peak_joint_err=" << retry.peakPdAbsError
                          << " retry_max_link_w=" << retry.maxLinkPreIntegrationAngularSpeed
                          << '\n';
                printHeldContacts("first", firstAttempt);
                printHeldContacts("retry", retry);
            }
        }
    }

    diagnostics.warmStartResets = impl_->totalWarmStartResets;
    diagnostics.retries = impl_->totalRetries;
    diagnostics.rollbackCount = impl_->totalRollbacks;
    diagnostics.heldStateCount = impl_->totalHeldStates;
    diagnostics.unsupportedIslandCount = impl_->totalUnsupportedIslands;
    diagnostics.totalStepTimeMs = elapsedMs(stepStart);
    if (diagnostics.status == ProximalStepStatus::Healthy
        || diagnostics.status == ProximalStepStatus::RecoveredRetry) {
        impl_->recordCommandStreamAccepted(dt, servoTargets);
    } else if (diagnostics.status == ProximalStepStatus::HeldLastGood
               || diagnostics.status == ProximalStepStatus::UnsupportedIsland) {
        const char* reason = diagnostics.failureReason == ProximalFailureReason::SpeedLimit
            ? "speed_limit"
            : "held_last_good";
        impl_->finishCommandStream(reason);
    }
    return diagnostics.status == ProximalStepStatus::Healthy
        || diagnostics.status == ProximalStepStatus::RecoveredRetry;
}

} // namespace minphys3d::demo
