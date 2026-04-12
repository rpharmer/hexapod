#pragma once

#include "demo/frame_sink.hpp"
#include "demo/scenes.hpp"
#include "minphys3d/math/vec3.hpp"

#include <cstddef>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace minphys3d::demo {

/// Applies a named interactive preset for UDP/visual debugging (sink, UDP host/port, gravity,
/// realtime pacing, frame budget, built-in model vs JSON path). `name_lower` must be ASCII lower-case.
/// If `selection_note_out` is non-null, on success it is filled with a short multi-line description
/// (scene summary + what to watch for visually).
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
    std::string* selection_note_out = nullptr);

void PrintInteractivePresetCatalog(std::ostream& out);

/// Table order matches `presets` / `ApplyInteractivePreset` (canonical key per row, already lower-case ASCII).
std::vector<std::string> ListInteractivePresetCanonicalKeysLowercase();

/// Index into `ListInteractivePresetCanonicalKeysLowercase()` for a preset name or alias, if known.
std::optional<std::size_t> InteractivePresetCatalogIndexForKeyLowercase(const std::string& name_lower);

} // namespace minphys3d::demo
