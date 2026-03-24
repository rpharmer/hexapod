#pragma once

#include <cstddef>
#include <vector>

#include "calibration_probe.hpp"

bool hasDebouncedContact(const std::vector<ProbeDestinationSample>& samples,
                         std::size_t index,
                         int debounce_samples);
