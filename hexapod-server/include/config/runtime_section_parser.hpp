#ifndef RUNTIME_SECTION_PARSER_HPP
#define RUNTIME_SECTION_PARSER_HPP

#include <memory>

#include <toml.hpp>

#include "hexapod-server.hpp"
#include "logger.hpp"

namespace runtime_section_parser {

bool parseRuntimeSection(const toml::value& root,
                         ParsedToml& out,
                         std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace runtime_section_parser

#endif // RUNTIME_SECTION_PARSER_HPP
