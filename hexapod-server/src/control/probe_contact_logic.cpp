#include "probe_contact_logic.hpp"

bool hasDebouncedContact(const std::vector<ProbeDestinationSample>& samples,
                         std::size_t index,
                         int debounce_samples) {
    if (debounce_samples <= 1) {
        return samples[index].foot_contact;
    }
    if (index + 1 < static_cast<std::size_t>(debounce_samples)) {
        return false;
    }
    for (int k = 0; k < debounce_samples; ++k) {
        if (!samples[index - static_cast<std::size_t>(k)].foot_contact) {
            return false;
        }
    }
    return true;
}
