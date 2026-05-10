#pragma once

namespace minphys3d {

/// Internal scalar type for minphys3d dynamics and collision (CPU-side).
/// UDP / `physics_sim_protocol` remain float32; widen/narrow at serve I/O only.
using Real = double;

constexpr Real kEpsilon = 1e-9;

} // namespace minphys3d
