#pragma once

#include <string>

#include "geometry_config.hpp"

namespace geometry_profile_service {

bool preview(const HexapodGeometry& candidate, std::string* error = nullptr);
bool apply(std::string* error = nullptr);
bool rollback(std::string* error = nullptr);
bool persist(std::string* error = nullptr);

const HexapodGeometry* previewSnapshot();

} // namespace geometry_profile_service
