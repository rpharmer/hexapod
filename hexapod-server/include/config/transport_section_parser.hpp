#ifndef TRANSPORT_SECTION_PARSER_HPP
#define TRANSPORT_SECTION_PARSER_HPP

#include <toml.hpp>

#include "hexapod-server.hpp"

namespace transport_section_parser {

bool parseTransportSection(const toml::value& root, ParsedToml& out, bool required);

} // namespace transport_section_parser

#endif // TRANSPORT_SECTION_PARSER_HPP
