#pragma once

#include "types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <ostream>
#include <vector>

// Test instrumentation only. All positions refer to the FK foot point; this
// is not a claim that FK height is collision-sphere bottom clearance.
namespace swing_census {
struct Sample {
    double time_s{};
    bool stance{true};
    bool contact{};
    double body_z{};
    Vec3 rotation_z{0, 0, 1};
    Vec3 foot_body{};
    Vec3 command_body{};
    double foot_world_z{};
};
inline double dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
struct Budget {
    double body{}, rotation{}, joint{}, measured{}, command{}, closure{};
};
inline Budget budget(const Sample& start, const Sample& s) {
    Budget b;
    b.body = s.body_z - start.body_z;
    // Exact ordered finite difference: rotation at the entry configuration,
    // followed by joint motion at the current orientation (interaction here).
    b.rotation = dot(s.rotation_z - start.rotation_z, start.foot_body);
    b.joint = dot(s.rotation_z, s.foot_body - start.foot_body);
    b.measured = s.foot_world_z - start.foot_world_z;
    b.command = s.body_z + dot(s.rotation_z, s.command_body) - start.foot_world_z;
    b.closure = b.measured - b.body - b.rotation - b.joint;
    return b;
}
struct Event {
    int leg{};
    double start_s{}, duration_ms{}, liftoff_ms{-1};
    bool complete{}, left_censored{};
    int recontacts{};
    double measured_peak_m{-1e30}, command_peak_m{-1e30}, peak_closure_m{};
    Budget at_command_peak{}, at_measured_peak{};
};
class Census {
public:
    void update(int leg, const Sample& s, bool valid = true) {
        auto& a = active_[leg];
        if (!valid) {
            close(leg, s.time_s, false);
            have_[leg] = false;
            return;
        }
        if (s.stance) {
            close(leg, s.time_s, true);
        } else {
            if (!a) {
                starts_[leg] = have_[leg] ? previous_[leg] : s;
                current_[leg] = Event{};
                auto& e = current_[leg];
                e.leg = leg;
                e.start_s = s.time_s;
                e.left_censored = !have_[leg];
                airborne_[leg] = false;
                a = true;
            }
            auto& e = current_[leg];
            if (!s.contact && !airborne_[leg]) {
                e.liftoff_ms = 1000 * (s.time_s - e.start_s);
                airborne_[leg] = true;
            } else if (s.contact && airborne_[leg] && !previous_[leg].contact) {
                ++e.recontacts;
            }
            const auto b = budget(starts_[leg], s);
            if (b.command > e.command_peak_m) {
                e.command_peak_m = b.command;
                e.at_command_peak = b;
            }
            if (b.measured > e.measured_peak_m) {
                e.measured_peak_m = b.measured;
                e.at_measured_peak = b;
            }
            e.peak_closure_m = std::max(e.peak_closure_m, std::abs(b.closure));
        }
        previous_[leg] = s;
        have_[leg] = true;
    }
    void finish(double time_s) {
        for (int leg = 0; leg < 6; ++leg) close(leg, time_s, false);
    }
    const std::vector<Event>& events() const { return events_; }
    void json(std::ostream& o) const {
        o << ",\"swing_event_schema\":1,\"swing_events\":[";
        bool comma = false;
        auto emitBudget = [&](const Budget& b) {
            o << "{\"body_dz_m\":" << b.body << ",\"rotation_dz_m\":" << b.rotation
              << ",\"joint_dz_m\":" << b.joint << ",\"measured_dz_m\":" << b.measured
              << ",\"command_dz_m\":" << b.command << ",\"closure_m\":" << b.closure << '}';
        };
        for (const auto& e : events_) {
            if (comma) o << ',';
            comma = true;
            o << "{\"leg\":" << e.leg << ",\"start_s\":" << e.start_s
              << ",\"duration_ms\":" << e.duration_ms << ",\"liftoff_ms\":" << e.liftoff_ms
              << ",\"complete\":" << (e.complete ? "true" : "false")
              << ",\"left_censored\":" << (e.left_censored ? "true" : "false")
              << ",\"recontacts\":" << e.recontacts
              << ",\"measured_peak_m\":" << e.measured_peak_m
              << ",\"command_peak_m\":" << e.command_peak_m
              << ",\"peak_closure_m\":" << e.peak_closure_m << ",\"at_command_peak\":";
            emitBudget(e.at_command_peak);
            o << ",\"at_measured_peak\":";
            emitBudget(e.at_measured_peak);
            o << '}';
        }
        o << ']';
    }
private:
    void close(int leg, double time_s, bool complete) {
        if (!active_[leg]) return;
        auto& e = current_[leg];
        e.duration_ms = 1000 * (time_s - e.start_s);
        e.complete = complete;
        events_.push_back(e);
        active_[leg] = false;
    }
    std::array<bool, 6> have_{}, active_{}, airborne_{};
    std::array<Sample, 6> starts_{}, previous_{};
    std::array<Event, 6> current_{};
    std::vector<Event> events_;
};
} // namespace swing_census
