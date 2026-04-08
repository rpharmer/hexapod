#pragma once

#include "subsystems.hpp"

namespace minphys3d::core_internal {

struct ContactPipelineContext {
    const ContactSolverContext& solverContext;
};

class ContactPipeline {
public:
    void BuildManifolds(const ContactPipelineContext& context) const;
};

} // namespace minphys3d::core_internal
