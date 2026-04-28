#include "demo/interactive_terminal.hpp"
#include "demo/scene_json.hpp"
#include "demo/scenes.hpp"
#include "demo/serve_mode.hpp"

#include "minphys3d/math/vec3.hpp"

#include <charconv>
#include <iostream>
#include <string>

namespace {

bool ParsePositiveInt(const char* text, int& out_value) {
    const char* end = text;
    while (*end != '\0') {
        ++end;
    }
    int value = 0;
    const auto result = std::from_chars(text, end, value);
    if (result.ec != std::errc{} || result.ptr != end || value <= 0) {
        return false;
    }
    out_value = value;
    return true;
}

bool ParseUdpPort(const char* text, int& out_value) {
    if (!ParsePositiveInt(text, out_value)) {
        return false;
    }
    return out_value <= 65535;
}

void PrintUsage(std::ostream& out) {
    out << "Usage: hexapod-physics-sim [options]\n"
           "  --sink dummy|udp          Frame sink (default: dummy)\n"
           "  --model default|hexapod   Built-in scene when not using --scene-file\n"
           "  --scene-file PATH         Load minphys JSON (see assets/scenes/examples/; schema 1–2)\n"
           "  --solver-iterations N     Solver iterations per physics step (hexapod demo; default: 40)\n"
           "  --udp-host HOST           UDP destination host (default: 127.0.0.1)\n"
           "  --udp-port PORT           UDP destination port (default: 9870)\n"
           "  --frames N                Number of frames to simulate (default: 1200)\n"
           "  --realtime                Pace the main loop to wall clock (~60 Hz) for visual debugging\n"
           "  --zero-gravity            Zero gravity vector (inspection / UDP preview)\n"
           "  --interactive, -i         Terminal REPL ( presets ,  preset NAME ,  l ,  r , …)\n"
           "  --autonext                With -i: after each full background run, apply the next catalog preset\n"
           "  --autonext-run            With -i: same as --autonext and start the next run automatically\n"
           "  --serve                   UDP physics server (hexapod scene, step-on-command)\n"
           "  --serve-port PORT         UDP listen port for --serve (default: 9871)\n"
           "  --scene-file PATH + --serve  Append obstacle bodies/joints from minphys JSON into serve mode\n"
           "  --sink udp + --serve      Also stream minphys scene UDP for hexapod-opengl-visualiser\n"
            "                            (same as demo: --udp-host / --udp-port, default 127.0.0.1:9870)\n"
           "  --serve-preview-stride N  With UDP preview: emit preview every N physics steps (default: 1)\n"
           "  -h, --help                Show this help\n";
}

} // namespace

int main(int argc, char** argv) {
    minphys3d::demo::SinkKind sink_kind = minphys3d::demo::SinkKind::Dummy;
    minphys3d::demo::SceneModel scene_model = minphys3d::demo::SceneModel::Default;
    int frame_count = 1200;
    bool realtime_playback = false;
    bool zero_gravity = false;
    bool interactive = false;
    bool interactive_autonext = false;
    bool interactive_autonext_run = false;
    bool serve_mode = false;
    int serve_port = 9871;
    int serve_preview_stride = 1;
    int solver_iterations = minphys3d::demo::kHexapodPoseHoldBenchmarkSolverIterations;
    std::string scene_file;
    std::string udp_host = "127.0.0.1";
    int udp_port = 9870;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sink") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --sink (expected dummy|udp)\n";
                return 1;
            }
            const std::string value = argv[++i];
            if (value == "dummy") {
                sink_kind = minphys3d::demo::SinkKind::Dummy;
            } else if (value == "udp") {
                sink_kind = minphys3d::demo::SinkKind::Udp;
            } else {
                std::cerr << "Unsupported sink: " << value << " (expected dummy|udp)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--model") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --model (expected default|hexapod)\n";
                return 1;
            }
            const std::string value = argv[++i];
            if (value == "default") {
                scene_model = minphys3d::demo::SceneModel::Default;
            } else if (value == "hexapod") {
                scene_model = minphys3d::demo::SceneModel::Hexapod;
            } else {
                std::cerr << "Unsupported model: " << value << " (expected default|hexapod)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--scene-file") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --scene-file\n";
                return 1;
            }
            scene_file = argv[++i];
            continue;
        }
        if (arg == "--solver-iterations") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --solver-iterations (expected positive integer)\n";
                return 1;
            }
            if (!ParsePositiveInt(argv[++i], solver_iterations)) {
                std::cerr << "Invalid --solver-iterations value (expected positive integer)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--udp-host") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --udp-host\n";
                return 1;
            }
            udp_host = argv[++i];
            continue;
        }
        if (arg == "--udp-port") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --udp-port\n";
                return 1;
            }
            if (!ParseUdpPort(argv[++i], udp_port)) {
                std::cerr << "Invalid --udp-port (expected integer 1..65535)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--frames") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --frames (expected positive integer)\n";
                return 1;
            }
            if (!ParsePositiveInt(argv[++i], frame_count)) {
                std::cerr << "Invalid --frames value (expected positive integer)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--realtime") {
            realtime_playback = true;
            continue;
        }
        if (arg == "--zero-gravity") {
            zero_gravity = true;
            continue;
        }
        if (arg == "--interactive" || arg == "-i") {
            interactive = true;
            continue;
        }
        if (arg == "--autonext-run") {
            interactive_autonext_run = true;
            interactive_autonext = true;
            continue;
        }
        if (arg == "--autonext") {
            interactive_autonext = true;
            continue;
        }
        if (arg == "-h" || arg == "--help") {
            PrintUsage(std::cout);
            return 0;
        }
        if (arg == "--serve") {
            serve_mode = true;
            continue;
        }
        if (arg == "--serve-port") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --serve-port\n";
                return 1;
            }
            if (!ParseUdpPort(argv[++i], serve_port)) {
                std::cerr << "Invalid --serve-port (expected integer 1..65535)\n";
                return 1;
            }
            continue;
        }
        if (arg == "--serve-preview-stride") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --serve-preview-stride\n";
                return 1;
            }
            if (!ParsePositiveInt(argv[++i], serve_preview_stride)) {
                std::cerr << "Invalid --serve-preview-stride (expected positive integer)\n";
                return 1;
            }
            continue;
        }

        std::cerr << "Unknown argument: " << arg << "\n";
        PrintUsage(std::cerr);
        return 1;
    }

    const minphys3d::Vec3 gravity = zero_gravity ? minphys3d::Vec3{0.0f, 0.0f, 0.0f}
                                                 : minphys3d::Vec3{0.0f, -9.81f, 0.0f};

    if (serve_mode) {
        if (serve_port < 1 || serve_port > 65535) {
            std::cerr << "Invalid serve port\n";
            return 1;
        }
        return minphys3d::demo::RunPhysicsServeMode(
            static_cast<std::uint16_t>(serve_port),
            sink_kind,
            udp_host,
            udp_port,
            scene_file,
            serve_preview_stride);
    }

    if (interactive) {
        minphys3d::demo::InteractiveSessionSeed seed;
        seed.sink_kind = sink_kind;
        seed.scene_model = scene_model;
        seed.frame_count = frame_count;
        seed.realtime_playback = realtime_playback;
        seed.gravity = gravity;
        seed.scene_json_path = scene_file;
        seed.udp_host = udp_host;
        seed.udp_port = udp_port;
        seed.autonext_preset = interactive_autonext;
        seed.autonext_auto_run = interactive_autonext_run;
        return minphys3d::demo::RunInteractiveTerminalSession(seed);
    }

    if (!scene_file.empty()) {
        return minphys3d::demo::RunPhysicsDemoFromJsonFile(
            scene_file,
            sink_kind,
            frame_count,
            realtime_playback,
            gravity,
            udp_host,
            udp_port);
    }

    return minphys3d::demo::RunPhysicsDemo(
        sink_kind,
        scene_model,
        frame_count,
        realtime_playback,
        gravity,
        udp_host,
        udp_port,
        nullptr,
        solver_iterations);
}
