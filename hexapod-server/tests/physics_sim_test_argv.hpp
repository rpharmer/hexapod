#pragma once

#include <cstdlib>
#include <cstring>

namespace physics_sim_test_argv {

/**
 * Parse argv: `--emit-metrics-json`, `--limits-manifest <path>`, first non-option token = sim exe.
 * Falls back to `HEXAPOD_PHYSICS_SIM_EXE` for sim.
 */
inline void parse(int argc, char** argv, bool& out_emit_metrics_json, const char*& out_sim_exe) {
    out_emit_metrics_json = false;
    out_sim_exe = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--emit-metrics-json") == 0) {
            out_emit_metrics_json = true;
        } else if (std::strcmp(argv[i], "--limits-manifest") == 0 && i + 1 < argc) {
            ++i;
        } else if (argv[i][0] != '\0' && argv[i][0] != '-') {
            out_sim_exe = argv[i];
        }
    }
    if (out_sim_exe == nullptr || out_sim_exe[0] == '\0') {
        out_sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
}

} // namespace physics_sim_test_argv
