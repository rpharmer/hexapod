#include "demo/interactive_terminal.hpp"

#include "demo/demo_run_control.hpp"
#include "demo/interactive_presets.hpp"
#include "demo/scene_json.hpp"
#include "demo/scenes.hpp"

#include <charconv>
#include <cctype>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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

void PrintBuiltinScenarioHint(SceneModel model) {
    if (model == SceneModel::Hexapod) {
        std::cout << "  Scene: Built-in articulated hexapod with servo legs.\n"
                  << "  Watch for: Chassis lowers until feet find the plane; legs cycle contacts while servos track "
                     "targets.\n";
    } else {
        std::cout << "  Scene: Built-in classic demo (boxes, sphere, hinge and distance joints).\n"
                  << "  Watch for: Upper box + hinge swing; sphere on rope-like distance constraint; impacts stay "
                     "crisp without obvious tunneling.\n";
    }
}

void PrintJsonScenarioHint() {
    std::cout << "  Scene: Loaded from minphys JSON (bodies, shapes, joints per file).\n"
              << "  Watch for: Motion matches the file intent—contacts, joint limits, and stacking or sliding should "
                 "look stable.\n";
}

void PrintJsonClearHint() {
    std::cout << "  Next run uses the built-in scenario from  1  /  2  until you load JSON or apply a preset.\n";
}

void PrintHelp() {
    std::cout
        << "Commands:\n"
        << "  r, run     Run one pass with current settings (built-in or JSON file).\n"
        << "              Runs in the background unless pause mode is on (then one synchronous frame).\n"
        << "  pause      Toggle pause mode: when on,  r  advances exactly 1 frame (UDP frame capture).\n"
        << "  step [N]   One-shot run of N frames (default 1); does not change the  f  frame budget.\n"
        << "  stop       Cancel a background run started with  r / run .\n"
        << "  wait       Block until the background run finishes (no cancel).\n"
        << "  sim-pause  Freeze physics in a background run (cooperative; use sim-resume ).\n"
        << "  sim-resume Resume after sim-pause .\n"
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
        << "  preset NAME   Load a curated UDP/visual preset (many aliases; see  presets ).\n"
        << "  presets      Print the full catalog of preset names and descriptions.\n"
        << "  autonext on|off     After each full background run, apply the next catalog preset (UDP visual tour).\n"
        << "  autonext-run on|off Also start the next  r  after autonext (only when pause_mode is off).\n"
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
    bool pause_mode,
    bool autonext_preset,
    bool autonext_auto_run) {
    std::cout << "[settings] ";
    if (!scene_json_path.empty()) {
        std::cout << "scenario=file:" << scene_json_path;
    } else {
        std::cout << "scenario=" << ModelLabel(model);
    }
    std::cout << "  sink=" << SinkLabel(sink) << "  udp=" << udp_host << ":" << udp_port
              << "  gravity=" << (GravityIsOff(gravity) ? "off" : "earth") << "  (" << gravity.x << "," << gravity.y
              << "," << gravity.z << ")" << "  realtime=" << (realtime ? "on" : "off") << "  frames=" << frames
              << "  pause_mode=" << (pause_mode ? "on" : "off") << "  autonext=" << (autonext_preset ? "on" : "off")
              << "  autonext-run=" << (autonext_auto_run ? "on" : "off") << "\n";
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
    bool autonext_preset = seed.autonext_preset;
    bool autonext_auto_run = seed.autonext_auto_run;
    int autonext_catalog_index = -1;

    std::optional<std::future<int>> sim_job;
    std::unique_ptr<DemoRunControl> sim_control;

    const auto run_demo_blocking = [&](int frame_budget, DemoRunControl* ctrl) -> int {
        if (!scene_json_path.empty()) {
            return RunPhysicsDemoFromJsonFile(
                scene_json_path, sink, frame_budget, realtime, gravity, udp_host, udp_port, ctrl);
        }
        return RunPhysicsDemo(sink, model, frame_budget, realtime, gravity, udp_host, udp_port, ctrl);
    };

    const auto launch_background_sim = [&]() {
        if (sim_job.has_value()) {
            return;
        }
        const int budget = frames;
        sim_control = std::make_unique<DemoRunControl>();
        sim_control->realtime_pacing.store(realtime, std::memory_order_relaxed);
        DemoRunControl* ctrl = sim_control.get();
        const std::string path_copy = scene_json_path;
        const SinkKind sink_copy = sink;
        const SceneModel model_copy = model;
        const minphys3d::Vec3 gravity_copy = gravity;
        const std::string host_copy = udp_host;
        const int port_copy = udp_port;
        const bool realtime_arg = realtime;
        sim_job.emplace(std::async(std::launch::async, [=]() -> int {
            if (!path_copy.empty()) {
                return RunPhysicsDemoFromJsonFile(
                    path_copy, sink_copy, budget, realtime_arg, gravity_copy, host_copy, port_copy, ctrl);
            }
            return RunPhysicsDemo(
                sink_copy, model_copy, budget, realtime_arg, gravity_copy, host_copy, port_copy, ctrl);
        }));
        std::cout << "[interactive] background run started ( stop | wait | sim-pause | sim-resume | t )\n";
    };

    const auto try_finalize_background_job = [&](bool blocking) -> bool {
        if (!sim_job.has_value()) {
            return false;
        }
        using namespace std::chrono;
        if (!blocking) {
            if (sim_job->wait_for(0s) != std::future_status::ready) {
                return false;
            }
        } else {
            sim_job->wait();
        }
        const bool cancelled =
            sim_control != nullptr && sim_control->cancel_requested.load(std::memory_order_acquire);
        (void)sim_job->get();
        sim_job.reset();
        sim_control.reset();

        std::cout << "[interactive] background simulation finished\n";
        if (!cancelled && autonext_preset) {
            const std::vector<std::string> keys = ListInteractivePresetCanonicalKeysLowercase();
            if (!keys.empty()) {
                std::size_t next_index = 0;
                if (autonext_catalog_index >= 0) {
                    next_index =
                        (static_cast<std::size_t>(autonext_catalog_index) + 1u) % keys.size();
                }
                autonext_catalog_index = static_cast<int>(next_index);
                std::string preset_note;
                if (ApplyInteractivePreset(
                        keys[next_index],
                        model,
                        sink,
                        gravity,
                        realtime,
                        frames,
                        scene_json_path,
                        udp_host,
                        udp_port,
                        &preset_note)) {
                    std::cout << "[interactive] autonext -> preset " << keys[next_index] << "\n"
                              << preset_note << "\n";
                }
                if (autonext_auto_run && !pause_mode) {
                    launch_background_sim();
                }
            }
        }
        PrintSettings(
            model,
            sink,
            gravity,
            realtime,
            frames,
            scene_json_path,
            udp_host,
            udp_port,
            pause_mode,
            autonext_preset,
            autonext_auto_run);
        return true;
    };

    const auto poll_sim_job = [&]() { (void)try_finalize_background_job(false); };

    const auto stop_background_sim = [&]() {
        if (sim_control != nullptr) {
            sim_control->cancel_requested.store(true, std::memory_order_release);
        }
        if (sim_job.has_value()) {
            sim_job->wait();
            (void)sim_job->get();
            sim_job.reset();
        }
        sim_control.reset();
    };

    std::cout << "================================================================================\n"
              << " hexapod-physics-sim — interactive terminal\n"
              << "================================================================================\n"
              << "Built-ins: 1 / 2   ·   JSON: l path   ·   then  r  to run.  h  for commands.\n\n";
    PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
    std::cout << "\n";

    for (;;) {
        poll_sim_job();
        std::cout << "demo> " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) {
            std::cout << "\n[interactive] stdin closed; exiting.\n";
            stop_background_sim();
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
            stop_background_sim();
            std::cout << "[interactive] bye.\n";
            return 0;
        }
        if (lower == "h" || lower == "?" || lower == "help") {
            PrintHelp();
            continue;
        }
        if (lower == "presets" || lower == "catalog" || lower == "scenarios") {
            PrintInteractivePresetCatalog(std::cout);
            continue;
        }
        if (lower == "p") {
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "pause") {
            pause_mode = !pause_mode;
            std::cout << "[interactive] pause_mode -> " << (pause_mode ? "on (each r runs 1 frame)" : "off") << "\n";
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "step" || lower.rfind("step ", 0) == 0) {
            if (sim_job.has_value()) {
                std::cout << "[interactive] stop the background run first ( stop )\n";
                continue;
            }
            std::string rest = lower == "step" ? std::string{} : line.substr(5);
            TrimInPlace(rest);
            int burst = 1;
            if (!rest.empty() && !ParsePositiveInt(rest, burst)) {
                std::cout << "[interactive] step [N] expects positive integer N (default 1)\n";
                continue;
            }
            const int rc = run_demo_blocking(burst, nullptr);
            if (rc != 0) {
                std::cout << "[interactive] step returned " << rc << "\n";
            } else {
                std::cout << "[interactive] stepped " << burst << " frame(s)\n";
            }
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "stop" || lower == "abort") {
            if (!sim_job.has_value()) {
                std::cout << "[interactive] no background simulation is running\n";
                continue;
            }
            stop_background_sim();
            std::cout << "[interactive] stopped.\n";
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "wait") {
            if (!sim_job.has_value()) {
                std::cout << "[interactive] no background simulation is running\n";
                continue;
            }
            (void)try_finalize_background_job(true);
            continue;
        }
        if (lower == "sim-pause" || lower == "hold") {
            if (sim_control == nullptr) {
                std::cout << "[interactive] no background simulation is running\n";
                continue;
            }
            sim_control->pause_requested.store(true, std::memory_order_release);
            std::cout << "[interactive] sim-pause requested (physics will hold after the current frame)\n";
            continue;
        }
        if (lower == "sim-resume" || lower == "continue") {
            if (sim_control == nullptr) {
                std::cout << "[interactive] no background simulation is running\n";
                continue;
            }
            sim_control->pause_requested.store(false, std::memory_order_release);
            std::cout << "[interactive] sim-resume\n";
            continue;
        }
        if (lower == "r" || lower == "run") {
            if (sim_job.has_value()) {
                std::cout << "[interactive] a run is already in progress ( stop  or  wait )\n";
                continue;
            }
            const int budget = pause_mode ? 1 : frames;
            if (pause_mode) {
                const int rc = run_demo_blocking(budget, nullptr);
                if (rc != 0) {
                    std::cout << "[interactive] run returned " << rc << "\n";
                }
            } else {
                launch_background_sim();
            }
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (line == "1") {
            autonext_catalog_index = -1;
            model = SceneModel::Default;
            scene_json_path.clear();
            std::cout << "[interactive] scenario -> classic (built-in)\n";
            PrintBuiltinScenarioHint(SceneModel::Default);
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (line == "2") {
            autonext_catalog_index = -1;
            model = SceneModel::Hexapod;
            scene_json_path.clear();
            std::cout << "[interactive] scenario -> hexapod (built-in)\n";
            PrintBuiltinScenarioHint(SceneModel::Hexapod);
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower.rfind("preset ", 0) == 0) {
            if (sim_job.has_value()) {
                std::cout << "[interactive] stop or wait for the background run before applying a preset\n";
                continue;
            }
            std::string name = line.substr(7);
            TrimInPlace(name);
            for (char& c : name) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            std::string preset_note;
            if (!ApplyInteractivePreset(
                    name,
                    model,
                    sink,
                    gravity,
                    realtime,
                    frames,
                    scene_json_path,
                    udp_host,
                    udp_port,
                    &preset_note)) {
                std::cout << "[interactive] unknown preset \"" << name << "\" ( presets  for the full list)\n";
                continue;
            }
            std::cout << "[interactive] preset -> " << name << "\n" << preset_note << "\n";
            if (const std::optional<std::size_t> idx = InteractivePresetCatalogIndexForKeyLowercase(name)) {
                autonext_catalog_index = static_cast<int>(*idx);
            }
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
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
            autonext_catalog_index = -1;
            std::cout << "[interactive] json scene -> " << scene_json_path << "\n";
            PrintJsonScenarioHint();
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "l" || lower == "load") {
            scene_json_path.clear();
            autonext_catalog_index = -1;
            std::cout << "[interactive] cleared JSON scene (built-in 1/2 in effect)\n";
            PrintJsonClearHint();
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
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
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "s") {
            sink = (sink == SinkKind::Dummy) ? SinkKind::Udp : SinkKind::Dummy;
            std::cout << "[interactive] sink -> " << SinkLabel(sink) << "\n";
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "g") {
            gravity = GravityIsOff(gravity) ? kEarthGravity : kZeroGravity;
            std::cout << "[interactive] gravity -> " << (GravityIsOff(gravity) ? "off" : "earth") << "\n";
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower == "t") {
            realtime = !realtime;
            if (sim_control != nullptr) {
                sim_control->realtime_pacing.store(realtime, std::memory_order_relaxed);
            }
            std::cout << "[interactive] realtime pacing -> " << (realtime ? "on" : "off") << "\n";
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }
        if (lower.rfind("autonext-run ", 0) == 0) {
            std::string v = lower.substr(13);
            TrimInPlace(v);
            if (v == "on") {
                autonext_auto_run = true;
                autonext_preset = true;
                std::cout << "[interactive] autonext-run on (preset autonext enabled)\n";
            } else if (v == "off") {
                autonext_auto_run = false;
                std::cout << "[interactive] autonext-run off\n";
            } else {
                std::cout << "[interactive] usage: autonext-run on|off\n";
                continue;
            }
            PrintSettings(
                model,
                sink,
                gravity,
                realtime,
                frames,
                scene_json_path,
                udp_host,
                udp_port,
                pause_mode,
                autonext_preset,
                autonext_auto_run);
            continue;
        }
        if (lower == "autonext") {
            std::cout << "[interactive] autonext=" << (autonext_preset ? "on" : "off") << "  autonext-run="
                      << (autonext_auto_run ? "on" : "off") << "\n";
            continue;
        }
        if (lower.rfind("autonext ", 0) == 0) {
            std::string v = lower.substr(9);
            TrimInPlace(v);
            if (v == "on") {
                autonext_preset = true;
                std::cout << "[interactive] autonext on\n";
            } else if (v == "off") {
                autonext_preset = false;
                autonext_auto_run = false;
                std::cout << "[interactive] autonext off (autonext-run cleared)\n";
            } else {
                std::cout << "[interactive] usage: autonext on|off\n";
                continue;
            }
            PrintSettings(
                model,
                sink,
                gravity,
                realtime,
                frames,
                scene_json_path,
                udp_host,
                udp_port,
                pause_mode,
                autonext_preset,
                autonext_auto_run);
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
            PrintSettings(
        model,
        sink,
        gravity,
        realtime,
        frames,
        scene_json_path,
        udp_host,
        udp_port,
        pause_mode,
        autonext_preset,
        autonext_auto_run);
            continue;
        }

        std::cout << "[interactive] unknown command \"" << line << "\" (try  h )\n";
    }
}

} // namespace minphys3d::demo
