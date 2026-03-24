#ifndef GEOMETRY_SECTION_PARSER_HPP
#define GEOMETRY_SECTION_PARSER_HPP

#include <toml.hpp>

#include "hexapod-server.hpp"

namespace geometry_section_parser {

void parseGeometrySection(const toml::value& root, ParsedToml& out);

} // namespace geometry_section_parser

#endif // GEOMETRY_SECTION_PARSER_HPP
