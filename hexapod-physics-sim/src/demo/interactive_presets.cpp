#include "demo/interactive_presets.hpp"

#include <cstdint>
#include <cstring>
#include <ostream>
#include <optional>
#include <sstream>
#include <string_view>
#include <vector>

namespace minphys3d::demo {
namespace {

const minphys3d::Vec3 kEarthG{0.0f, -9.81f, 0.0f};
const minphys3d::Vec3 kZeroG{0.0f, 0.0f, 0.0f};

enum class PresetKind : std::uint8_t {
    BuiltinClassic,
    BuiltinHex,
    Json,
};

struct PresetEntry {
    const char* keys_pipe;
    const char* blurb;
    const char* visual_hint;
    PresetKind kind;
    SceneModel builtin_model;
    const char* json_path;
    bool zero_gravity;
    bool realtime_off;
    int frames;
};

// Paths are resolved by scene_json (cwd may be repo root, hexapod-physics-sim/, or build/).
static const PresetEntry kPresetTable[] = {
    {"hex-viz|hex|hex-live",
        "Built-in hexapod, Earth gravity, UDP pacing",
        "Chassis settles on the plane; six legs cycle through contacts; servo-driven joints should track without exploding.",
        PresetKind::BuiltinHex,
        SceneModel::Hexapod,
        nullptr,
        false,
        false,
        960},
    {"hex-zero|hex-0g|hex-zerog",
        "Built-in hexapod, zero gravity float",
        "No weight: post-step servo snap is disabled for this preset so the rig stays coherent; expect slow drift + joint PD, not explosive snapping.",
        PresetKind::BuiltinHex,
        SceneModel::Hexapod,
        nullptr,
        true,
        false,
        720},
    {"classic-viz|classic|boxes-demo|default-viz",
        "Built-in boxes + sphere + joints",
        "Upper box + hinge swings; distance joint pulls the sphere; expect stable stacking on the lower box and clean impacts.",
        PresetKind::BuiltinClassic,
        SceneModel::Default,
        nullptr,
        false,
        false,
        900},
    {"classic-zero|classic-0g",
        "Built-in classic scene, zero gravity",
        "Same topology as classic but weightless: bodies should float slowly with joint constraints still visible.",
        PresetKind::BuiltinClassic,
        SceneModel::Default,
        nullptr,
        true,
        false,
        600},
    {"classic-soak|classic-fast",
        "Classic: no wall-clock pacing (UDP as fast as CPU)",
        "Very fast frame stream: use to stress throughput; motion should match classic-viz but time-skip between UDP frames is large.",
        PresetKind::BuiltinClassic,
        SceneModel::Default,
        nullptr,
        false,
        true,
        4800},

    {"json-stack|stack|vis-stack-minimal",
        "examples: stack_minimal (plane + box + sphere)",
        "Sphere arcs in from the side; expect a bounce off the box top then ground roll—no interpenetration through the box.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/examples/stack_minimal.json",
        false,
        false,
        840},
    {"json-compound|compound|vis-compound",
        "examples: compound_preview",
        "Compound hull tumbles; child shapes should stay rigidly attached (no splitting) while corners contact the plane.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/examples/compound_preview.json",
        false,
        false,
        720},
    {"json-pendulum|pendulum|distance-demo",
        "examples: joints_distance (schema 2)",
        "Small sphere tethered by distance joint to a static anchor; expect springy oscillation, not infinite stretch.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/examples/joints_distance.json",
        false,
        false,
        960},

    {"vis-sphere-drop|col-sphere-plane",
        "visual: sphere drop on plane",
        "Single bounce then roll-out; contact normal should stay roughly vertical on first hit.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_sphere_drop.json",
        false,
        false,
        900},
    {"vis-box-tower|stack-boxes-4",
        "visual: four stacked boxes",
        "Tower may wobble then settle; watch vertical stack stability and corner contacts without sinking.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_box_tower.json",
        false,
        false,
        960},
    {"vis-cylinder-roll|col-cylinder-plane",
        "visual: cylinder rolls on plane",
        "Tilted cylinder should pick up rolling motion; caps and barrel contacts alternate as it moves.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_cylinder_roll.json",
        false,
        false,
        1020},
    {"vis-two-cylinders|col-cyl-cyl",
        "visual: cylinder resting on static cylinder",
        "Upper cylinder finds a rest pose on the barrel; expect multi-point style contact patch, no jitter explosion.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_two_cylinders.json",
        false,
        false,
        1080},
    {"vis-halfcyl-plane|col-hc-plane",
        "visual: half-cylinder on plane (open +Z)",
        "Flat cut may kiss the ground while curved shell carries weight; half-space z>=0 should be respected visually.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_halfcyl_plane.json",
        false,
        false,
        960},
    {"vis-halfcyl-box|col-hc-box",
        "visual: half-cylinder vs static box",
        "Curved shell slides along the wall; expect sliding contact without the half-cylinder tunneling through the box.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_halfcyl_box.json",
        false,
        false,
        1020},
    {"vis-caps-cyl|col-cap-cyl",
        "visual: capsule vs static cylinder",
        "Capsule barrel or cap hits the cylinder; rounded ends should produce smooth deflection rather than snagging.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_caps_cyl.json",
        false,
        false,
        1080},
    {"vis-sph-cyl|col-sphere-cyl",
        "visual: sphere vs static cylinder",
        "Sphere should wrap contacts around the curved surface and fall away cleanly after the skim.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_sph_cyl.json",
        false,
        false,
        960},
    {"vis-box-cyl|col-box-cyl",
        "visual: box vs static cylinder",
        "Box corner/edge against cylinder barrel; watch for stable resting or tipping without deep penetration.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_box_cyl.json",
        false,
        false,
        1020},
    {"vis-hc-cyl|col-hc-cyl",
        "visual: half-cylinder vs full cylinder",
        "Specialized manifold case: curved half meets full barrel—contacts should spread when overlap is deep.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_hc_cyl.json",
        false,
        false,
        1200},
    {"vis-sphere-rain|rain-spheres",
        "visual: several spheres falling",
        "Multiple impacts in quick succession; spheres should not pass through each other when colliding mid-air or on ground.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_sphere_rain.json",
        false,
        false,
        900},
    {"vis-row-slide|slide-row",
        "visual: row of boxes with lateral velocity",
        "Boxes slide and may collide in-line; expect friction to slow them with occasional corner bumps.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_row_slide.json",
        false,
        false,
        840},
    {"vis-seesaw|hinge-seesaw",
        "visual: hinge seesaw plank",
        "Plank pivots about the hinge axis; ends should bob opposite phase with the fulcrum staying fixed.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_seesaw.json",
        false,
        false,
        1200},
    {"vis-dangle-dist|soft-rope",
        "visual: distance joint dangle",
        "Soft spring rope: sphere swings under the anchor with damped oscillation; rope length should look bounded.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_dangle_dist.json",
        false,
        false,
        1200},
    {"vis-servo-arm|servo-demo",
        "visual: servo between static base and arm",
        "Servo drives relative angle; arm should seek target with limited overshoot if gains are sane.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_servo_arm.json",
        false,
        false,
        1500},
    {"vis-ball-socket|balljoint-demo",
        "visual: ball-socket chain feel",
        "Sphere pivots about the anchor on the post; expect free swing but no detachment from the joint.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_ball_socket.json",
        false,
        false,
        1200},
    {"vis-fixed-pair|weld-demo",
        "visual: fixed joint rigid pair",
        "Two boxes move as one rigid assembly after the weld; initial velocity should translate/rotate the pair together.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_fixed_pair.json",
        false,
        false,
        900},
    {"vis-prismatic-slide|prism-slide",
        "visual: prismatic joint slide",
        "Upper block slides along axis relative to base; motion should stay colinear with the prismatic axis.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_prismatic_slide.json",
        false,
        false,
        1200},
    {"vis-compound-tumble|compound-roll",
        "visual: compound shape with initial spin",
        "Compound tumbles with initial angular velocity; outline should stay rigid (child shapes locked).",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_compound_tumble.json",
        false,
        false,
        1020},
    {"vis-caps-triple|capsule-pile",
        "visual: three capsules interacting",
        "Capsules jostle; watch capsule–capsule rolling and resting poses without interpenetration.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_caps_triple.json",
        false,
        false,
        1080},
    {"vis-hc-pair|hc-vs-hc",
        "visual: two half-cylinders",
        "Curved shells meet; expect multi-contact feel along the overlap region when pressed together.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_hc_pair.json",
        false,
        false,
        1200},
    {"vis-spin-box|gyro-box",
        "visual: box with high angular velocity",
        "Precessing/spinning box on plane; edges may skate—angular velocity should decay smoothly with friction.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_spin_box.json",
        false,
        false,
        900},
    {"vis-hinge-chain|chain-hinges",
        "visual: two hinges chain of boxes",
        "Double pendulum-like chain; inner link should swing relative to tower and outer link relative to inner.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_hinge_chain.json",
        false,
        false,
        1320},
    {"vis-ice-slide|low-friction-slide",
        "visual: low-friction plane + sliding box",
        "Box keeps speed longer; expect long glide distance and gentle yaw changes compared to normal friction.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_ice_slide.json",
        false,
        false,
        960},
    {"vis-bounce-tower|bouncy-stack",
        "visual: high restitution small stack",
        "Bouncy spheres: small hops after impacts; tower may separate vertically on first hits.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_bounce_tower.json",
        false,
        false,
        900},
    {"vis-tilt-stack|leaning-tower",
        "visual: tilted boxes settle",
        "Initially tilted boxes should rotate toward stable resting poses; possible partial collapse.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_tilt_stack.json",
        false,
        false,
        1080},
    {"vis-corner-hit|edge-impact",
        "visual: fast box hits vertical edge",
        "Fast inbound box deflects off the L-shaped statics; look for a sharp bounce without tunneling the walls.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_corner_hit.json",
        false,
        false,
        840},
    {"vis-cyl-tower|cylinder-stack",
        "visual: two dynamic cylinders stacked",
        "Upper cylinder balances on lower; watch rocking until damped rest or roll-off.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_cyl_tower.json",
        false,
        false,
        1200},
    {"vis-motor-hinge|hinge-motor-swing",
        "visual: hinge with motor enabled",
        "Motor torque should drive continuous rotation about hinge axis; plank speed should trend toward motor speed.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_motor_hinge.json",
        false,
        false,
        1500},
    {"vis-sleep-candidates|gentle-settle",
        "visual: many bodies gently settling (sleep debug)",
        "Tiny boxes should come to rest and eventually sleep (no micro-bounce); good for checking sleep thresholds.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_sleep_candidates.json",
        false,
        false,
        1800},
    {"vis-offset-stack|jenga-ish",
        "visual: offset stacked boxes topple",
        "COM offset causes tipping; expect dramatic topple or staggered falls depending on friction.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_offset_stack.json",
        false,
        false,
        1020},
    {"vis-caps-wall|capsule-wall-hit",
        "visual: capsule into wall of boxes",
        "Capsule slams the static wall; should compress slightly on impact then rebound or slide along faces.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_caps_wall.json",
        false,
        false,
        960},
    {"vis-sphere-chain|sphere-train",
        "visual: spheres in a line with impulse",
        "Lead sphere impulse propagates; expect Newton’s-cradle-like collisions along the row.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_sphere_chain.json",
        false,
        false,
        900},
    {"vis-heavy-light|mass-ratio",
        "visual: heavy box + light sphere collision",
        "Tiny fast sphere should bounce off the massive block with large velocity change on the sphere, small on the block.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_heavy_light.json",
        false,
        false,
        960},
    {"vis-angled-plank|plank-drop",
        "visual: long thin box drops at angle",
        "Thin plank flutters/rotates while falling; landing may flex into a stable diagonal rest.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_angled_plank.json",
        false,
        false,
        900},
    {"vis-double-pendulum|two-rope",
        "visual: two distance joints (double dangle)",
        "Two-stage springy pendulum: inner mass swings, outer follows with phase lag; no infinite elongation.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_double_pendulum.json",
        false,
        false,
        1320},
    {"vis-rolling-spheres|billiard-ish",
        "visual: spheres rolling into each other",
        "Rolling with spin; glancing collisions should curve paths modestly on the plane.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_rolling_spheres.json",
        false,
        false,
        1080},
    {"vis-static-mess|many-statics",
        "visual: one dynamic among many static obstacles",
        "Sphere threads obstacles; expect multiple sequential contacts and deflections without passing through walls.",
        PresetKind::Json,
        SceneModel::Default,
        "assets/scenes/visual/vis_static_mess.json",
        false,
        false,
        1200},
};

bool KeysMatch(const char* keys_pipe, const std::string& needle_lower) {
    const std::string_view keys(keys_pipe);
    std::size_t start = 0;
    for (;;) {
        const std::size_t bar = keys.find('|', start);
        const std::size_t end = (bar == std::string_view::npos) ? keys.size() : bar;
        if (start < keys.size() && end > start) {
            const std::string_view tok = keys.substr(start, end - start);
            if (tok.size() == needle_lower.size()
                && std::memcmp(tok.data(), needle_lower.data(), tok.size()) == 0) {
                return true;
            }
        }
        if (bar == std::string_view::npos) {
            break;
        }
        start = bar + 1;
    }
    return false;
}

std::string CanonicalPresetKey(const char* keys_pipe) {
    const std::string_view keys(keys_pipe);
    const std::size_t bar = keys.find('|');
    return std::string(bar == std::string_view::npos ? keys : keys.substr(0, bar));
}

std::string FormatPresetSelectionNote(const PresetEntry& e) {
    std::ostringstream out;
    out << "  [preset: " << CanonicalPresetKey(e.keys_pipe) << "]\n"
        << "  Scene: " << e.blurb << "\n"
        << "  Watch for: " << e.visual_hint;
    return out.str();
}

} // namespace

bool ApplyInteractivePreset(
    const std::string& name_lower,
    SceneModel& model,
    SinkKind& sink,
    minphys3d::Vec3& gravity,
    bool& realtime,
    int& frames,
    std::string& scene_json_path,
    std::string& udp_host,
    int& udp_port,
    std::string* selection_note_out) {

    for (const PresetEntry& e : kPresetTable) {
        if (!KeysMatch(e.keys_pipe, name_lower)) {
            continue;
        }
        sink = SinkKind::Udp;
        udp_host = "127.0.0.1";
        udp_port = 9870;
        realtime = !e.realtime_off;
        gravity = e.zero_gravity ? kZeroG : kEarthG;
        frames = e.frames;
        scene_json_path.clear();
        if (e.kind == PresetKind::Json) {
            scene_json_path = e.json_path;
            model = SceneModel::Default;
        } else if (e.kind == PresetKind::BuiltinHex) {
            model = SceneModel::Hexapod;
        } else {
            model = SceneModel::Default;
        }
        if (selection_note_out != nullptr) {
            *selection_note_out = FormatPresetSelectionNote(e);
        }
        return true;
    }
    return false;
}

void PrintInteractivePresetCatalog(std::ostream& out) {
    out << "Interactive presets ( preset <name> ). First name in each line is canonical; others are aliases.\n"
        << "Defaults: UDP 127.0.0.1:9870, Earth gravity, realtime pacing on (except classic-soak).\n\n";
    for (const PresetEntry& e : kPresetTable) {
        out << "  " << e.keys_pipe << "\n      " << e.blurb << "  [" << e.frames << " frames]\n"
            << "      Watch for: " << e.visual_hint << "\n";
    }
    out << "\nTip: start hexapod-opengl-visualiser on the same UDP port, then  preset hex-viz  and  r .\n"
        << "Autonext:  autonext on  cycles the preset table after each full background run; add  autonext-run on  "
           "to start the next  r  automatically.\n";
}

std::vector<std::string> ListInteractivePresetCanonicalKeysLowercase() {
    std::vector<std::string> out;
    for (const PresetEntry& e : kPresetTable) {
        out.push_back(CanonicalPresetKey(e.keys_pipe));
    }
    return out;
}

std::optional<std::size_t> InteractivePresetCatalogIndexForKeyLowercase(const std::string& name_lower) {
    std::size_t i = 0;
    for (const PresetEntry& e : kPresetTable) {
        if (KeysMatch(e.keys_pipe, name_lower)) {
            return i;
        }
        ++i;
    }
    return std::nullopt;
}

} // namespace minphys3d::demo
