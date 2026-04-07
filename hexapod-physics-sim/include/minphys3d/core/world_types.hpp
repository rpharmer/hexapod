#pragma once

#include <cstddef>
#include <cstdint>

namespace minphys3d {

struct ContactKey {
    std::uint32_t loBody = 0;
    std::uint32_t hiBody = 0;
    std::uint64_t featureKey = 0;

    bool operator==(const ContactKey& other) const {
        return loBody == other.loBody && hiBody == other.hiBody && featureKey == other.featureKey;
    }
};

struct ManifoldKey {
    std::uint32_t loBody = 0;
    std::uint32_t hiBody = 0;
    std::uint8_t manifoldType = 0;

    bool operator==(const ManifoldKey& other) const {
        return loBody == other.loBody && hiBody == other.hiBody && manifoldType == other.manifoldType;
    }
};

struct PersistentPointKey {
    ManifoldKey manifold{};
    std::uint64_t featureKey = 0;
    std::uint8_t ordinal = 0;

    bool operator==(const PersistentPointKey& other) const {
        return manifold == other.manifold && featureKey == other.featureKey && ordinal == other.ordinal;
    }
};

struct ContactKeyHash {
    std::size_t operator()(const ContactKey& key) const noexcept {
        std::size_t seed = static_cast<std::size_t>(key.loBody);
        seed ^= static_cast<std::size_t>(key.hiBody) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= static_cast<std::size_t>(key.featureKey) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= static_cast<std::size_t>(key.featureKey >> 32u) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        return seed;
    }
};

struct ManifoldKeyHash {
    std::size_t operator()(const ManifoldKey& key) const noexcept {
        std::size_t seed = static_cast<std::size_t>(key.loBody);
        seed ^= static_cast<std::size_t>(key.hiBody) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= static_cast<std::size_t>(key.manifoldType) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        return seed;
    }
};

struct PersistentPointKeyHash {
    std::size_t operator()(const PersistentPointKey& key) const noexcept {
        std::size_t seed = ManifoldKeyHash{}(key.manifold);
        seed ^= static_cast<std::size_t>(key.featureKey) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= static_cast<std::size_t>(key.featureKey >> 32u) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        seed ^= static_cast<std::size_t>(key.ordinal) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        return seed;
    }
};

} // namespace minphys3d
