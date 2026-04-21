#include "demo/process_resource_diagnostics.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testParseProcStatmLine() {
    std::uint64_t size_pages = 0;
    std::uint64_t resident_pages = 0;
    if (!expect(resource_monitoring::parseProcStatmLine("4096 1024 77", size_pages, resident_pages),
                "valid /proc/self/statm line should parse")) {
        return false;
    }
    if (!expect(size_pages == 4096, "statm size pages should parse") ||
        !expect(resident_pages == 1024, "statm resident pages should parse")) {
        return false;
    }

    size_pages = 0;
    resident_pages = 0;
    return expect(!resource_monitoring::parseProcStatmLine("4096", size_pages, resident_pages),
                  "truncated /proc/self/statm line should fail");
}

bool testResourceLoggingWrapper() {
    std::FILE* tmp = std::tmpfile();
    if (!expect(tmp != nullptr, "tmpfile should open for logging wrapper test")) {
        return false;
    }

    minphys3d::demo::ProcessResourceDiagnosticsState state{};
    state.emit_interval = std::chrono::milliseconds{0};
    minphys3d::demo::MaybeLogProcessResourceSnapshot(tmp, state, "[resource]");
    std::fflush(tmp);
    std::rewind(tmp);

    char buffer[512]{};
    std::string contents;
    while (std::fgets(buffer, sizeof(buffer), tmp) != nullptr) {
        contents += buffer;
    }
    std::fclose(tmp);

    return expect(contents.find("[resource] process_resource=") != std::string::npos,
                  "resource logging wrapper should emit the expected prefix") &&
           expect(contents.find("cpu_percent=") != std::string::npos,
                  "resource logging wrapper should include cpu percent") &&
           expect(contents.find("rss_bytes=") != std::string::npos,
                  "resource logging wrapper should include rss bytes") &&
           expect(contents.find("vms_bytes=") != std::string::npos,
                  "resource logging wrapper should include vms bytes");
}

} // namespace

int main() {
    if (!testParseProcStatmLine()) {
        return EXIT_FAILURE;
    }
    if (!testResourceLoggingWrapper()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
