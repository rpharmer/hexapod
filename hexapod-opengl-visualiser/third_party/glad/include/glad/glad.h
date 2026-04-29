/**
 * Compatibility shim: older code and ImGui expect <glad/glad.h> while glad2
 * generates <glad/gl.h>. Map the legacy loader name to glad2's gladLoadGL.
 */
#pragma once

#include <glad/gl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy glad 1.x name used by some examples */
static inline int gladLoadGLLoader(GLADloadfunc load) { return gladLoadGL(load); }

#ifdef __cplusplus
}
#endif
