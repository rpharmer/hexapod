#ifndef TUNING_SECTION_PARSER_HPP
#define TUNING_SECTION_PARSER_HPP

#include <memory>

#include <toml.hpp>

#include "hexapod-server.hpp"
#include "logger.hpp"

namespace tuning_section_parser {

void parseTuningSection(const toml::value& root,
                        ParsedToml& out,
                        std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace tuning_section_parser

#endif // TUNING_SECTION_PARSER_HPP
