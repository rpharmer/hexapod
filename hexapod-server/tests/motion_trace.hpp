#pragma once
#include "replay_json.hpp"
#include <ostream>

// Diagnostic trace, not an executable command fixture. Some callers collect
// MotionSample rather than complete bus joint commands; only the debug FK
// and explicitly recorded state are used by the support/swing audit.
inline void writeMotionTrace(std::ostream& out, const replay_json::ReplayTelemetryRecord& r) {
    auto json = replay_json::serializeReplayTelemetryRecord(r);
    json.pop_back();
    out << json << ",\"trace_only\":true,\"planning_detail\":{";
    auto vectors = [&](const char* name, const auto& values) {
        out << '"' << name << "\":[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i) out << ',';
            out << '[' << values[i].x << ',' << values[i].y << ',' << values[i].z << ']';
        }
        out << ']';
    };
    vectors("planned_body_m", r.locomotion_debug.planned_leg_target_body_m);
    out << ',';
    vectors("pre_slew_body_m", r.locomotion_debug.pre_slew_fk_body_m);
    out << ',';
    vectors("post_clamp_body_m", r.locomotion_debug.post_clamp_fk_body_m);
    out << "}}\n";
}
