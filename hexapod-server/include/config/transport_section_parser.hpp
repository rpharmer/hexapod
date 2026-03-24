#ifndef TRANSPORT_SECTION_PARSER_HPP
#define TRANSPORT_SECTION_PARSER_HPP

#include <memory>

#include <toml.hpp>

#include "hexapod-server.hpp"
#include "logger.hpp"

namespace transport_section_parser {

bool parseTransportSection(const toml::value& root,
                           ParsedToml& out,
                           bool required,
                           std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace transport_section_parser

#endif // TRANSPORT_SECTION_PARSER_HPP
