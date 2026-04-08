#pragma once

// Narrowphase collision routines are implemented as World private methods
// in core/world_impl.hpp (SphereBox, BoxBox, CapsuleBox, etc.).

namespace minphys3d {

struct NarrowphaseTag {};

enum class ConvexDispatchRoute {
    SpecializedOnly,
    GenericGateThenSpecialized,
    GenericOnly,
};

enum class NarrowphaseDispatchPolicy {
    PreferSpecializedFastPaths,
    PreferGenericConvexCore,
};

} // namespace minphys3d
