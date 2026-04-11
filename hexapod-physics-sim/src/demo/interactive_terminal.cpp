#include "demo/interactive_terminal.hpp"

#include "demo/scene_json.hpp"

#include <charconv>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

namespace minphys3d::demo {
namespace {

const minphys3d::Vec3 kEarthGravity{0.0f, -9.81f, 0.0f};
const minphys3d::Vec3 kZeroGravity{0.0f, 0.0f, 0.0f};

void TrimInPlace(std::string& s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
        s.erase(s.begin());
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
        s.pop_back();
    }
}

bool ParsePositiveInt(std::string_view text, int& out_value) {
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    int value = 0;
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end || value <= 0) {
        return false;
    }
    out_value = value;
    return true;
}

bool GravityIsOff(const minphys3d::Vec3& g) {
    return minphys3d::LengthSquared(g) < 1e-12f;
}

const char* SinkLabel(SinkKind k) {
    switch (k) {
        case SinkKind::Dummy:
            return "dummy";
        case SinkKind::Udp:
            return "udp";
    }
    return "?";
}

const char* ModelLabel(SceneModel m) {
    switch (m) {
        case SceneModel::Default:
            return "classic (boxes + sphere + joints)";
        case SceneModel::Hexapod:
            return "hexapod";
    }
    return "?";
}

void PrintHelp() {
    std::cout
        << "Commands:\n"
        << "  r, run     Run one pass with current settings (built-in or JSON file).\n"
        << "  pause      Toggle pause mode: when on,  r  advances exactly 1 frame (UDP frame capture).\n"
        << "  step [N]   One-shot run of N frames (default 1); does not change the  f  frame budget.\n"
        << "  q, quit    Exit.\n"
        << "  1 / 2      Built-in scenario: classic or hexapod (clears JSON file mode).\n"
        << "  l PATH     Load a minphys JSON scene (see assets/scenes/examples/).\n"
        << "  l          Clear JSON path; next run uses built-in 1/2 scenario.\n"
        << "  load       Same as  l .\n"
        << "  u HOST PORT   UDP destination for the visualiser (e.g.  u 127.0.0.1 9870 ).\n"
        << "  s          Cycle frame sink dummy <-> udp.\n"
        << "  g          Toggle Earth gravity vs zero-G.\n"
        << "  t          Toggle realtime pacing (~60 Hz).\n"
        << "  f N / f    Set frame count (or prompt).\n"
        << "  preset NAME   hex-viz | hex-zero | json-stack | json-compound | json-pendulum\n"
        << "  p          Print settings.\n"
        << "  h, help    This list.\n";
}

void PrintSettings(
    SceneModel model,
    SinkKind sink,
    const minphys3d::Vec3& gravity,
    bool realtime,
    int frames,
    const std::string& scene_json_path,
    const std::string& udp_host,
    int udp_port,
    bool pause_mode) {
    std::cout << "[settings] ";
    if (!scene_json_path.empty()) {
        std::cout << "scenario=file:" << scene_json_path;
    } else {
        std::cout << "scenario=" << ModelLabel(model);
    }
    std::cout << "  sink=" << SinkLabel(sink) << "  udp=" << udp_host << ":" << udp_port
              << "  gravity=" << (GravityIsOff(gravity) ? "off" : "earth") << "  (" << gravity.x << "," << gravity.y
              << "," << gravity.z << ")" << "  realtime=" << (realtime ? "on" : "off") << "  frames=" << frames
              << "  pause_mode=" << (pause_mode ? "on" : "off") << "\n";
}

bool ApplyPreset(
    const std::string& name,
    SceneModel& model,
    SinkKind& sink,
    minphys3d::Vec3& gravity,
    bool& realtime,
    int& frames,
    std::string& scene_json_path,
    std::string& udp_host,
    int& udp_port) {
    if (name == "hex-viz") {
        model = SceneModel::Hexapod;
        scene_json_path.clear();
        sink = SinkKind::Udp;
        gravity = kEarthGravity;
        realtime = true;
        frames = 900;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        return true;
    }
    if (name == "hex-zero") {
        model = SceneModel::Hexapod;
        scene_json_path.clear();
        sink = SinkKind::Udp;
        gravity = kZeroGravity;
        realtime = true;
        frames = 500;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        return true;
    }
    if (name == "json-stack") {
        scene_json_path = "assets/scenes/examples/stack_minimal.json";
        sink = SinkKind::Udp;
        gravity = kEarthGravity;
        realtime = true;
        frames = 720;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        return true;
    }
    if (name == "json-compound") {
        scene_json_path = "assets/scenes/examples/compound_preview.json";
        sink = SinkKind::Udp;
        gravity = kEarthGravity;
        realtime = true;
        frames = 600;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        return true;
    }
    if (name == "json-pendulum") {
        scene_json_path = "assets/scenes/examples/joints_distance.json";
        sink = SinkKind::Udp;
        gravity = kEarthGravity;
        realtime = true;
        frames = 840;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        return true;
    }
    return false;
}

} // namespace

int RunInteractiveTerminalSession(const InteractiveSessionSeed& seed) {
    SceneModel model = seed.scene_model;
    SinkKind sink = seed.sink_kind;
    minphys3d::Vec3 gravity = seed.gravity;
    bool realtime = seed.realtime_playback;
    int frames = seed.frame_count;
    std::string scene_json_path = seed.scene_json_path;
    std::string udp_host = seed.udp_host;
    int udp_port = seed.udp_port;
    bool pause_mode = false;

    const auto executeRun = [&](int frame_budget) -> int {
        if (!scene_json_path.empty()) {
            return RunPhysicsDemoFromJsonFile(
                scene_json_path,
                sink,
                frame_budget,
                realtime,
                gravity,
                udp_host,
                udp_port);
        }
        return RunPhysicsDemo(sink, model, frame_budget, realtime, gravity, udp_host, udp_port);
    };

    std::cout << "================================================================================\n"
              << " hexapod-physics-sim — interactive terminal\n"
              << "================================================================================\n"
              << "Built-ins: 1 / 2   ·   JSON: l path   ·   then  r  to run.  h  for commands.\n\n";
    PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
    std::cout << "\n";

    for (;;) {
        std::cout << "demo> " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) {
            std::cout << "\n[interactive] stdin closed; exiting.\n";
            return 0;
        }
        TrimInPlace(line);
        if (line.empty()) {
            continue;
        }

        std::string lower = line;
        for (char& c : lower) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }

        if (lower == "q" || lower == "quit" || lower == "exit") {
            std::cout << "[interactive] bye.\n";
            return 0;
        }
        if (lower == "h" || lower == "?" || lower == "help") {
            PrintHelp();
            continue;
        }
        if (lower == "p") {
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "pause") {
            pause_mode = !pause_mode;
            std::cout << "[interactive] pause_mode -> " << (pause_mode ? "on (each r runs 1 frame)" : "off") << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "step" || lower.rfind("step ", 0) == 0) {
            std::string rest = lower == "step" ? std::string{} : line.substr(5);
            TrimInPlace(rest);
            int burst = 1;
            if (!rest.empty() && !ParsePositiveInt(rest, burst)) {
                std::cout << "[interactive] step [N] expects positive integer N (default 1)\n";
                continue;
            }
            const int rc = executeRun(burst);
            if (rc != 0) {
                std::cout << "[interactive] step returned " << rc << "\n";
            } else {
                std::cout << "[interactive] stepped " << burst << " frame(s)\n";
            }
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "r" || lower == "run") {
            const int budget = pause_mode ? 1 : frames;
            const int rc = executeRun(budget);
            if (rc != 0) {
                std::cout << "[interactive] run returned " << rc << "\n";
            }
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (line == "1") {
            model = SceneModel::Default;
            scene_json_path.clear();
            std::cout << "[interactive] scenario -> classic (built-in)\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (line == "2") {
            model = SceneModel::Hexapod;
            scene_json_path.clear();
            std::cout << "[interactive] scenario -> hexapod (built-in)\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower.rfind("preset ", 0) == 0) {
            std::string name = line.substr(7);
            TrimInPlace(name);
            for (char& c : name) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            if (!ApplyPreset(name, model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port)) {
                std::cout << "[interactive] unknown preset \"" << name << "\" (try  h )\n";
                continue;
            }
            std::cout << "[interactive] preset -> " << name << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower.rfind("l ", 0) == 0 || lower.rfind("load ", 0) == 0) {
            const std::size_t skip = (lower.rfind("load ", 0) == 0) ? 5u : 2u;
            std::string path = line.substr(skip);
            TrimInPlace(path);
            if (path.empty()) {
                std::cout << "[interactive] missing path after l / load\n";
                continue;
            }
            scene_json_path = path;
            std::cout << "[interactive] json scene -> " << scene_json_path << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "l" || lower == "load") {
            scene_json_path.clear();
            std::cout << "[interactive] cleared JSON scene (built-in 1/2 in effect)\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower.rfind("u ", 0) == 0 || lower.rfind("udp ", 0) == 0) {
            const std::size_t skip = (lower.rfind("udp ", 0) == 0) ? 4u : 2u;
            std::istringstream iss(line.substr(skip));
            std::string host;
            int port = 0;
            if (!(iss >> host >> port)) {
                std::cout << "[interactive] usage: u HOST PORT   (e.g.  u 127.0.0.1 9870 )\n";
                continue;
            }
            if (port < 1 || port > 65535) {
                std::cout << "[interactive] port must be 1..65535\n";
                continue;
            }
            udp_host = std::move(host);
            udp_port = port;
            std::cout << "[interactive] udp -> " << udp_host << ":" << udp_port << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "s") {
            sink = (sink == SinkKind::Dummy) ? SinkKind::Udp : SinkKind::Dummy;
            std::cout << "[interactive] sink -> " << SinkLabel(sink) << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "g") {
            gravity = GravityIsOff(gravity) ? kEarthGravity : kZeroGravity;
            std::cout << "[interactive] gravity -> " << (GravityIsOff(gravity) ? "off" : "earth") << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (lower == "t") {
            realtime = !realtime;
            std::cout << "[interactive] realtime pacing -> " << (realtime ? "on" : "off") << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }
        if (!line.empty() && (line[0] == 'f' || line[0] == 'F')) {
            std::string arg = line.size() > 1 ? line.substr(1) : std::string{};
            TrimInPlace(arg);
            if (arg.empty()) {
                std::cout << "frames> " << std::flush;
                if (!std::getline(std::cin, arg)) {
                    std::cout << "\n[interactive] stdin closed; exiting.\n";
                    return 0;
                }
                TrimInPlace(arg);
            }
            int n = 0;
            if (!ParsePositiveInt(arg, n)) {
                std::cout << "[interactive] expected positive integer for frames\n";
                continue;
            }
            frames = n;
            std::cout << "[interactive] frames -> " << frames << "\n";
            PrintSettings(model, sink, gravity, realtime, frames, scene_json_path, udp_host, udp_port, pause_mode);
            continue;
        }

        std::cout << "[interactive] unknown command \"" << line << "\" (try  h )\n";
    }
}

} // namespace minphys3d::demo
