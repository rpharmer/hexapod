#ifndef RUNTIME_SECTION_PARSER_HPP
#define RUNTIME_SECTION_PARSER_HPP

#include <toml.hpp>

#include "hexapod-server.hpp"

namespace runtime_section_parser {

bool parseRuntimeSection(const toml::value& root, ParsedToml& out);

} // namespace runtime_section_parser

#endif // RUNTIME_SECTION_PARSER_HPP
