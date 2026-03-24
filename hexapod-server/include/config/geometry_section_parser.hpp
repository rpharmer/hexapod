#ifndef GEOMETRY_SECTION_PARSER_HPP
#define GEOMETRY_SECTION_PARSER_HPP

#include <memory>

#include <toml.hpp>

#include "hexapod-server.hpp"
#include "logger.hpp"

namespace geometry_section_parser {

void parseGeometrySection(const toml::value& root,
                          ParsedToml& out,
                          std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace geometry_section_parser

#endif // GEOMETRY_SECTION_PARSER_HPP
