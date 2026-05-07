#pragma once

#include "locomotion_metrics.hpp"

#include <iostream>
#include <string>

namespace physics_sim_metrics {

/** One JSON object line: suite, name, passed, limits_applied (object), metrics (object). */
inline void emitLine(const std::string& suite,
                     const std::string& name,
                     const bool passed,
                     const std::string& limits_applied_object,
                     const std::string& metrics_object) {
    std::cout << "{\"suite\":\"" << locomotion_test::jsonEscape(suite) << "\",\"name\":\""
              << locomotion_test::jsonEscape(name) << "\",\"passed\":" << (passed ? "true" : "false")
              << ",\"limits_applied\":" << limits_applied_object << ",\"metrics\":" << metrics_object << "}\n";
}

} // namespace physics_sim_metrics
