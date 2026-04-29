#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace visualiser::parsing {

std::size_t SkipWhitespace(std::string_view payload, std::size_t index);
std::optional<std::string> ExtractStringField(std::string_view payload, std::string_view key);
std::optional<std::string_view> ExtractStructuredField(
    std::string_view payload,
    std::string_view key,
    char open_char,
    char close_char);
std::optional<std::string_view> ExtractObjectField(std::string_view payload, std::string_view key);
std::optional<std::string_view> ExtractArrayField(std::string_view payload, std::string_view key);
std::optional<std::uint32_t> ExtractUintField(std::string_view payload, std::string_view key);
std::optional<float> ExtractFloatField(std::string_view payload, std::string_view key);
std::optional<std::array<float, 3>> ExtractFloat3Field(std::string_view payload, std::string_view key);
std::optional<std::array<float, 4>> ExtractFloat4Field(std::string_view payload, std::string_view key);
std::optional<int> ExtractIntField(std::string_view payload, std::string_view key);
std::optional<double> ExtractDoubleField(std::string_view payload, std::string_view key);
std::optional<bool> ExtractBoolField(std::string_view payload, std::string_view key);
std::optional<std::vector<float>> ExtractFloatArrayField(std::string_view payload, std::string_view key);

}  // namespace visualiser::parsing
