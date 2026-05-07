#include "test_limits_manifest.hpp"

#include "test_limits_manifest_paths.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_set>

namespace {

nlohmann::json g_root{};
bool g_loaded = false;
bool g_explicit = false;
std::unordered_set<std::string> g_warned_keys;

std::string warnKey(const std::string& suite,
                     const std::string& name,
                     const std::string& profile,
                     const std::string& key) {
    return suite + '\0' + name + '\0' + profile + '\0' + key;
}

/** Extract --limits-manifest <path> from argv; returns path and leaves argv semantics to caller. */
std::optional<std::string> limitsPathFromArgv(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--limits-manifest") == 0) {
            return std::string{argv[i + 1]};
        }
    }
    return std::nullopt;
}

std::optional<std::string> resolvePath(int argc, char** argv) {
    if (const auto p = limitsPathFromArgv(argc, argv)) {
        return p;
    }
    if (const char* env = std::getenv("HEXAPOD_TEST_LIMITS_MANIFEST")) {
        if (env[0] != '\0') {
            return std::string{env};
        }
    }
    {
        const std::string def{HEXAPOD_DEFAULT_LIMITS_MANIFEST};
        if (!def.empty() && std::filesystem::exists(def)) {
            return def;
        }
    }
    return std::nullopt;
}

bool mergeFlatPrimitives(nlohmann::json& into, const nlohmann::json& node) {
    if (!node.is_object()) {
        return false;
    }
    for (auto it = node.begin(); it != node.end(); ++it) {
        const std::string k = it.key();
        if (k == "_defaults" || k == "strict" || k == "loose" || k == "forward_like" || k == "lateral") {
            continue;
        }
        if (it.value().is_number() || it.value().is_boolean()) {
            into[k] = it.value();
        }
    }
    return true;
}

nlohmann::json mergedCase(const std::string& suite, const std::string& name, const std::string& profile) {
    nlohmann::json merged = nlohmann::json::object();
    const auto& suites = g_root["suites"];
    if (!suites.is_object() || !suites.contains(suite)) {
        return merged;
    }
    const auto& suiteNode = suites[suite];
    if (!suiteNode.is_object()) {
        return merged;
    }
    if (suiteNode.contains("_defaults") && suiteNode["_defaults"].is_object()) {
        mergeFlatPrimitives(merged, suiteNode["_defaults"]);
    }
    if (!suiteNode.contains(name) || !suiteNode[name].is_object()) {
        return merged;
    }
    const auto& caseNode = suiteNode[name];
    if (!profile.empty() && caseNode.contains(profile) && caseNode[profile].is_object()) {
        mergeFlatPrimitives(merged, caseNode[profile]);
    } else {
        mergeFlatPrimitives(merged, caseNode);
    }
    return merged;
}

} // namespace

namespace test_limits {

bool init(int argc, char** argv, std::string& error_out) {
    g_root = nlohmann::json::object();
    g_loaded = false;
    g_explicit = false;
    g_warned_keys.clear();

    const auto pathOpt = resolvePath(argc, argv);
    if (!pathOpt.has_value() || pathOpt->empty()) {
        return true;
    }
    const std::string& path = *pathOpt;
    g_explicit = limitsPathFromArgv(argc, argv).has_value() ||
                 (std::getenv("HEXAPOD_TEST_LIMITS_MANIFEST") != nullptr &&
                  std::getenv("HEXAPOD_TEST_LIMITS_MANIFEST")[0] != '\0');
    // Default repo-relative manifest (compile-time path) does not count as "explicit" for missing-key warnings.

    std::ifstream in(path);
    if (!in) {
        error_out = "limits manifest: cannot open file: " + path;
        return !g_explicit;
    }
    try {
        in >> g_root;
    } catch (const std::exception& ex) {
        error_out = std::string{"limits manifest: parse error: "} + ex.what();
        g_root = nlohmann::json::object();
        return !g_explicit;
    }

    if (!g_root.contains("schema_version") || !g_root["schema_version"].is_number_integer()) {
        error_out = "limits manifest: missing schema_version";
        g_root = nlohmann::json::object();
        return !g_explicit;
    }
    const int ver = g_root["schema_version"].get<int>();
    if (ver != 1) {
        error_out = "limits manifest: unsupported schema_version " + std::to_string(ver);
        g_root = nlohmann::json::object();
        return !g_explicit;
    }
    g_loaded = true;
    return true;
}

bool loaded() {
    return g_loaded;
}

double getDouble(const std::string& suite,
                 const std::string& name,
                 const std::string& profile,
                 const std::string& key,
                 const double fallback) {
    if (!g_loaded) {
        return fallback;
    }
    const nlohmann::json merged = mergedCase(suite, name, profile);
    if (!merged.contains(key)) {
        if (g_explicit) {
            const std::string wk = warnKey(suite, name, profile, key);
            if (!g_warned_keys.count(wk)) {
                g_warned_keys.insert(wk);
                std::cerr << "limits manifest: missing key \"" << key << "\" for " << suite << "/" << name;
                if (!profile.empty()) {
                    std::cerr << " profile=" << profile;
                }
                std::cerr << " (using compile default)\n";
            }
        }
        return fallback;
    }
    const auto& v = merged[key];
    if (v.is_number()) {
        return v.get<double>();
    }
    return fallback;
}

bool getBool(const std::string& suite,
             const std::string& name,
             const std::string& profile,
             const std::string& key,
             const bool fallback) {
    if (!g_loaded) {
        return fallback;
    }
    const nlohmann::json merged = mergedCase(suite, name, profile);
    if (!merged.contains(key)) {
        return fallback;
    }
    const auto& v = merged[key];
    if (v.is_boolean()) {
        return v.get<bool>();
    }
    if (v.is_number_integer()) {
        return v.get<int>() != 0;
    }
    return fallback;
}

std::size_t getSizeT(const std::string& suite,
                     const std::string& name,
                     const std::string& profile,
                     const std::string& key,
                     const std::size_t fallback) {
    const double v = getDouble(suite, name, profile, key, static_cast<double>(fallback));
    if (!std::isfinite(v) || v < 0.0) {
        return fallback;
    }
    return static_cast<std::size_t>(v + 0.5);
}

} // namespace test_limits
