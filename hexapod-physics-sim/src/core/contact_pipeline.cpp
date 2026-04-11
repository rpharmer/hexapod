#include "minphys3d/core/contact_pipeline.hpp"

namespace minphys3d::core_internal {

void ContactPipeline::BuildManifolds(const ContactPipelineContext& context) const {
    ContactSolver solver;
    solver.BuildManifolds(context.solverContext);
}

} // namespace minphys3d::core_internal
