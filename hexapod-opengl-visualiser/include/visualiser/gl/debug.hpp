#pragma once

namespace visualiser::gl {

/// Registers GL_KHR_debug callback when available (no-op in Release / if unsupported).
void SetupDebugCallback();

}  // namespace visualiser::gl
