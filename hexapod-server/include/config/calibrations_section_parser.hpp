#ifndef CALIBRATIONS_SECTION_PARSER_HPP
#define CALIBRATIONS_SECTION_PARSER_HPP

#include <string>
#include <tuple>
#include <vector>

#include <toml.hpp>

namespace calibrations_section_parser {

using CalibrationRow = std::tuple<std::string, int, int>;

bool parseCalibrationsSection(const toml::value& root, std::vector<CalibrationRow>& out);

} // namespace calibrations_section_parser

#endif // CALIBRATIONS_SECTION_PARSER_HPP
