#pragma once

#include <array>
#include <cstddef>

inline constexpr double kStanceProgressEdge = 0.05;
inline constexpr double kHighDutyFactor = 0.70;
inline constexpr std::size_t kTripodPlannedStanceCount = 3;
inline constexpr std::size_t kOverlapPlannedStanceCount = 4;

inline bool isHighDuty(const double duty_factor) {
    return duty_factor > kHighDutyFactor;
}

inline bool isMidPlannedStance(const bool planned,
                               const double phase,
                               const double duty_factor,
                               const double edge = kStanceProgressEdge) {
    if (!planned || duty_factor <= 0.0) {
        return false;
    }
    const double tau = phase / duty_factor;
    return tau > edge && tau < 1.0 - edge;
}

inline bool isOnsetPlannedStance(const bool planned,
                                 const bool previous_planned,
                                 const double phase,
                                 const double duty_factor,
                                 const double edge = kStanceProgressEdge) {
    if (!planned) {
        return false;
    }
    if (!previous_planned) {
        return true;
    }
    if (duty_factor <= 0.0) {
        return false;
    }
    return (phase / duty_factor) <= edge;
}

template <std::size_t N>
inline std::size_t plannedStanceCount(const std::array<bool, N>& planned) {
    std::size_t count = 0;
    for (const bool in_stance : planned) {
        count += in_stance ? 1U : 0U;
    }
    return count;
}

inline bool isTripodStanceFrame(const std::size_t planned_count) {
    return planned_count == kTripodPlannedStanceCount;
}

inline bool isOverlapStanceFrame(const std::size_t planned_count) {
    return planned_count >= kOverlapPlannedStanceCount;
}

// Legal tripod for plant-fidelity slices: the three planned feet are the only
// contacts, and none of those contacted stance feet are parked on plant `L`.
template <std::size_t N>
inline bool isCleanTripodFrame(const std::array<bool, N>& planned,
                               const std::array<bool, N>& contacts,
                               const std::size_t n_l_parked_contacted) {
    if (plannedStanceCount(planned) != kTripodPlannedStanceCount
        || plannedStanceCount(contacts) != kTripodPlannedStanceCount
        || n_l_parked_contacted != 0) {
        return false;
    }
    for (std::size_t i = 0; i < N; ++i) {
        if (planned[i] != contacts[i]) {
            return false;
        }
    }
    return true;
}
