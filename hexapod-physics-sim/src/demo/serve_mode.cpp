#include "demo/serve_mode.hpp"

#include "demo/frame_sink.hpp"
#include "demo/scene_json.hpp"
#include "demo/scenes.hpp"
#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/math/vec3.hpp"
#include "minphys3d/solver/types.hpp"
#include "physics_sim_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "serve_mode requires POSIX sockets (Linux/macOS)"
#endif

namespace minphys3d::demo {
namespace {

constexpr std::size_t kRecvBufBytes = 512;

std::array<std::uint32_t, 18> ServoJointIdsInWireOrder(const HexapodSceneObjects& scene) {
    return {
        scene.legs[0].bodyToCoxaJoint,   scene.legs[0].coxaToFemurJoint,   scene.legs[0].femurToTibiaJoint,
        scene.legs[1].bodyToCoxaJoint,   scene.legs[1].coxaToFemurJoint,   scene.legs[1].femurToTibiaJoint,
        scene.legs[2].bodyToCoxaJoint,   scene.legs[2].coxaToFemurJoint,   scene.legs[2].femurToTibiaJoint,
        scene.legs[3].bodyToCoxaJoint,   scene.legs[3].coxaToFemurJoint,   scene.legs[3].femurToTibiaJoint,
        scene.legs[4].bodyToCoxaJoint,   scene.legs[4].coxaToFemurJoint,   scene.legs[4].femurToTibiaJoint,
        scene.legs[5].bodyToCoxaJoint,   scene.legs[5].coxaToFemurJoint,   scene.legs[5].femurToTibiaJoint,
    };
}

Vec3 AverageUnitNormal(const Vec3& acc, const Vec3& n) {
    const Vec3 sum = acc + n;
    const float len = Length(sum);
    if (len <= 1.0e-6f) {
        return n;
    }
    return sum * (1.0f / len);
}

float WrapAngleRad(const float angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

float YawAboutSimUp(const Quat& q) {
    const float siny_cosp = 2.0f * (q.w * q.y + q.x * q.z);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<physics_sim::ObstacleFootprint> BuildObstacleFootprint(const Body& body) {
    if (body.shape == ShapeType::Plane) {
        return std::nullopt;
    }

    physics_sim::ObstacleFootprint footprint{};
    footprint.center_x = body.position.x;
    footprint.center_z = body.position.z;
    footprint.yaw_rad = WrapAngleRad(YawAboutSimUp(body.orientation));

    switch (body.shape) {
        case ShapeType::Box:
        case ShapeType::Compound:
            footprint.half_extent_x = body.halfExtents.x;
            footprint.half_extent_z = body.halfExtents.z;
            break;
        case ShapeType::Sphere:
            footprint.half_extent_x = body.radius;
            footprint.half_extent_z = body.radius;
            footprint.yaw_rad = 0.0f;
            break;
        case ShapeType::Capsule:
        case ShapeType::Cylinder:
        case ShapeType::HalfCylinder: {
            const float extent = std::max(body.radius, body.halfHeight + body.radius);
            footprint.half_extent_x = extent;
            footprint.half_extent_z = extent;
            footprint.yaw_rad = 0.0f;
            break;
        }
        default:
            return std::nullopt;
    }

    if (footprint.half_extent_x <= 0.0f || footprint.half_extent_z <= 0.0f) {
        return std::nullopt;
    }
    return footprint;
}

std::vector<std::uint32_t> CollectObstacleBodyIds(const World& world, const HexapodSceneObjects& scene) {
    std::vector<std::uint32_t> out{};
    std::vector<bool> robot_body_flags(world.GetBodyCount(), false);
    for (const std::uint32_t id : scene.body_ids) {
        if (id < robot_body_flags.size()) {
            robot_body_flags[id] = true;
        }
    }

    for (std::uint32_t body_id = 0; body_id < world.GetBodyCount(); ++body_id) {
        if (robot_body_flags[body_id]) {
            continue;
        }
        const Body& body = world.GetBody(body_id);
        if (body.shape == ShapeType::Plane) {
            continue;
        }
        if (BuildObstacleFootprint(body).has_value()) {
            out.push_back(body_id);
        }
    }
    return out;
}

void FillObstacleFootprints(const World& world,
                            const std::vector<std::uint32_t>& obstacle_body_ids,
                            physics_sim::StateResponse& rsp) {
    rsp.obstacle_count = 0;
    for (const std::uint32_t body_id : obstacle_body_ids) {
        if (rsp.obstacle_count >= physics_sim::kMaxObstacleFootprints) {
            break;
        }
        const auto footprint = BuildObstacleFootprint(world.GetBody(body_id));
        if (!footprint.has_value()) {
            continue;
        }
        rsp.obstacles[static_cast<std::size_t>(rsp.obstacle_count)] = *footprint;
        ++rsp.obstacle_count;
    }
}

void EmitObstacleBodies(FrameSink& sink, const World& world, const std::vector<std::uint32_t>& obstacle_body_ids) {
    for (const std::uint32_t body_id : obstacle_body_ids) {
        sink.emit_body(body_id, world.GetBody(body_id));
    }
}

void FillFootContactsFromManifolds(
    const World& world,
    const HexapodSceneObjects& scene,
    std::array<std::uint8_t, 6>& out_contact,
    std::array<std::array<float, 3>, 6>& out_normals) {
    out_contact.fill(0);
    for (auto& n : out_normals) {
        n = {0.0f, 1.0f, 0.0f};
    }
    std::array<Vec3, 6> accum{};
    std::array<int, 6> counts{};

    Vec3 plane_up{0.0f, 1.0f, 0.0f};
    const Body& plane_body = world.GetBody(scene.plane);
    if (plane_body.shape == ShapeType::Plane) {
        plane_up = Normalize(plane_body.planeNormal);
    }

    for (const Manifold& manifold : world.DebugManifolds()) {
        const bool a_is_plane = manifold.a == scene.plane;
        const bool b_is_plane = manifold.b == scene.plane;
        if (a_is_plane == b_is_plane) {
            continue;
        }
        const std::uint32_t dyn_id = a_is_plane ? manifold.b : manifold.a;
        Vec3 n = manifold.normal;
        if (Dot(n, plane_up) < 0.0f) {
            n = n * -1.0f;
        }
        for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
            if (scene.legs[leg].tibia != dyn_id) {
                continue;
            }
            if (manifold.contacts.empty()) {
                continue;
            }
            out_contact[leg] = 1;
            accum[leg] = AverageUnitNormal(accum[leg], n);
            ++counts[leg];
            break;
        }
    }
    for (std::size_t leg = 0; leg < 6; ++leg) {
        if (counts[leg] > 0) {
            const Vec3 u = Normalize(accum[leg]);
            out_normals[leg] = {u.x, u.y, u.z};
        }
    }
}

} // namespace

int RunPhysicsServeMode(std::uint16_t listen_port,
                        SinkKind preview_sink,
                        const std::string& preview_udp_host,
                        int preview_udp_port,
                        const std::string& scene_file) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[serve] socket() failed\n";
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "[serve] bind port " << listen_port << " failed\n";
        ::close(fd);
        return 1;
    }

    World world(Vec3{0.0f, -9.81f, 0.0f});
    HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    int solver_iterations = 8;

    if (!scene_file.empty()) {
        std::string error;
        int appended_joints = 0;
        if (!AppendWorldFromMinphysSceneJsonFile(
                scene_file, world, solver_iterations, error, &appended_joints)) {
            std::cerr << "[serve] failed to append scene file '" << scene_file << "': " << error << "\n";
            ::close(fd);
            return 1;
        }
        std::cout << "[serve] appended scene file '" << scene_file << "' joints=" << appended_joints << "\n";
    }

    const std::vector<std::uint32_t> obstacle_body_ids = CollectObstacleBodyIds(world, scene);

    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    joint_cfg.hingeAnchorBiasFactor = 0.25f;
    joint_cfg.hingeAnchorDampingFactor = 0.3f;
    world.SetJointSolverConfig(joint_cfg);

    const std::array<std::uint32_t, 18> wire_joints = ServoJointIdsInWireOrder(scene);
    std::array<float, 18> prev_angles{};
    for (std::size_t i = 0; i < wire_joints.size(); ++i) {
        prev_angles[i] = world.GetServoJointAngle(wire_joints[i]);
    }

    bool configured = false;
    std::vector<std::byte> recv_buf(kRecvBufBytes);

    std::unique_ptr<FrameSink> visual = MakeFrameSink(preview_sink, preview_udp_host, preview_udp_port);
    int preview_frame = 0;
    float preview_sim_time_s = 0.0f;

    std::cout << "[hexapod-physics-sim] serve mode UDP *:" << listen_port;
    if (preview_sink == SinkKind::Udp) {
        std::cout << "  preview UDP -> " << preview_udp_host << ":" << preview_udp_port;
    }
    if (!scene_file.empty()) {
        std::cout << "  scene_file=" << scene_file;
    }
    std::cout << "\n";

    for (;;) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        const ssize_t n = ::recvfrom(
            fd,
            reinterpret_cast<char*>(recv_buf.data()),
            recv_buf.size(),
            0,
            reinterpret_cast<sockaddr*>(&peer),
            &peer_len);
        if (n < 0) {
            std::cerr << "[serve] recvfrom failed\n";
            ::close(fd);
            return 1;
        }

        if (n < 1) {
            continue;
        }

        const auto* bytes = reinterpret_cast<const std::byte*>(recv_buf.data());
        const std::uint8_t mtype = static_cast<std::uint8_t>(*bytes);

        if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::ConfigCommand)) {
            physics_sim::ConfigCommand cmd{};
            if (!physics_sim::tryDecodeConfigCommand(bytes, static_cast<std::size_t>(n), cmd)) {
                continue;
            }
            world.SetGravity(Vec3{cmd.gravity[0], cmd.gravity[1], cmd.gravity[2]});
            solver_iterations = cmd.solver_iterations > 0 ? cmd.solver_iterations : 8;
            configured = true;

            physics_sim::ConfigAck ack{};
            ack.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::ConfigAck);
            ack.body_count = world.GetBodyCount();
            ack.joint_count = world.GetServoJointCount();
            (void)::sendto(
                fd,
                &ack,
                physics_sim::kConfigAckBytes,
                0,
                reinterpret_cast<sockaddr*>(&peer),
                peer_len);
            continue;
        }

        if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand)) {
            if (!configured) {
                continue;
            }
            physics_sim::StepCommand step{};
            if (!physics_sim::tryDecodeStepCommand(bytes, static_cast<std::size_t>(n), step)) {
                continue;
            }

            // sequence_id == 0: return current state without stepping or applying joint_targets.
            if (step.sequence_id == 0) {
                physics_sim::StateResponse rsp{};
                rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
                rsp.sequence_id = 0;

                const Body& chassis = world.GetBody(scene.body);
                rsp.body_position = {chassis.position.x, chassis.position.y, chassis.position.z};
                rsp.body_orientation = {
                    chassis.orientation.w,
                    chassis.orientation.x,
                    chassis.orientation.y,
                    chassis.orientation.z,
                };
                rsp.body_linear_velocity = {chassis.velocity.x, chassis.velocity.y, chassis.velocity.z};
                rsp.body_angular_velocity = {
                    chassis.angularVelocity.x,
                    chassis.angularVelocity.y,
                    chassis.angularVelocity.z,
                };

                for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                    rsp.joint_angles[i] = world.GetServoJointAngle(wire_joints[i]);
                    rsp.joint_velocities[i] = 0.0f;
                }
                FillFootContactsFromManifolds(world, scene, rsp.foot_contacts, rsp.foot_contact_normals);
                FillObstacleFootprints(world, obstacle_body_ids, rsp);

                (void)::sendto(
                    fd,
                    &rsp,
                    physics_sim::kStateResponseBytes,
                    0,
                    reinterpret_cast<sockaddr*>(&peer),
                    peer_len);
                continue;
            }

            if (step.dt_seconds <= 0.0f || !std::isfinite(step.dt_seconds)) {
                continue;
            }

            for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                ServoJoint& sj = world.GetServoJointMutable(wire_joints[i]);
                sj.targetAngle = step.joint_targets[i];
            }

            for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                prev_angles[i] = world.GetServoJointAngle(wire_joints[i]);
            }

            world.Step(step.dt_seconds, solver_iterations);

            physics_sim::StateResponse rsp{};
            rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
            rsp.sequence_id = step.sequence_id;

            const Body& chassis = world.GetBody(scene.body);
            rsp.body_position = {chassis.position.x, chassis.position.y, chassis.position.z};
            rsp.body_orientation = {
                chassis.orientation.w,
                chassis.orientation.x,
                chassis.orientation.y,
                chassis.orientation.z,
            };
            rsp.body_linear_velocity = {chassis.velocity.x, chassis.velocity.y, chassis.velocity.z};
            rsp.body_angular_velocity = {
                chassis.angularVelocity.x,
                chassis.angularVelocity.y,
                chassis.angularVelocity.z,
            };

            const float inv_dt = 1.0f / step.dt_seconds;
            for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                const float ang_after = world.GetServoJointAngle(wire_joints[i]);
                rsp.joint_angles[i] = ang_after;
                rsp.joint_velocities[i] = (ang_after - prev_angles[i]) * inv_dt;
                prev_angles[i] = ang_after;
            }

            FillFootContactsFromManifolds(world, scene, rsp.foot_contacts, rsp.foot_contact_normals);
            FillObstacleFootprints(world, obstacle_body_ids, rsp);

            (void)::sendto(
                fd,
                &rsp,
                physics_sim::kStateResponseBytes,
                0,
                reinterpret_cast<sockaddr*>(&peer),
                peer_len);

            preview_sim_time_s += step.dt_seconds;
            visual->begin_frame(preview_frame, preview_sim_time_s);
            EmitSceneBodies(*visual, world, scene);
            EmitObstacleBodies(*visual, world, obstacle_body_ids);
            visual->end_frame();
            ++preview_frame;
            continue;
        }
    }
}

} // namespace minphys3d::demo
