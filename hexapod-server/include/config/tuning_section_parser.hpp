#ifndef TUNING_SECTION_PARSER_HPP
#define TUNING_SECTION_PARSER_HPP

#include <toml.hpp>

#include "hexapod-server.hpp"

namespace tuning_section_parser {

void parseTuningSection(const toml::value& root, ParsedToml& out);

} // namespace tuning_section_parser

#endif // TUNING_SECTION_PARSER_HPP
