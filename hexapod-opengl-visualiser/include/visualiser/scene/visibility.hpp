#pragma once

#include <algorithm>

namespace visualiser::scene {

// Both the live and modular scene readers use the same shape/pose handshake:
// a pose-only UDP entity is not drawable until its static descriptor arrives.
template <typename EntityMap>
bool HasDrawableMeasuredGeometry(const EntityMap& entities) {
  return std::any_of(entities.begin(), entities.end(), [](const auto& entry) {
    const auto& entity = entry.second;
    using Shape = decltype(entity.shape);
    return entity.has_static && entity.has_frame &&
           entity.shape != Shape::kPlane && entity.shape != Shape::kUnknown;
  });
}

}  // namespace visualiser::scene
