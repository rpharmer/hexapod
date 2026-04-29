#include "visualiser/parsing/json_extract.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <sstream>

namespace visualiser::parsing {

std::size_t SkipWhitespace(std::string_view payload, std::size_t index) {
  while (index < payload.size()
         && std::isspace(static_cast<unsigned char>(payload[index])) != 0) {
    ++index;
  }
  return index;
}

std::optional<std::string> ExtractStringField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":\"";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find('"', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }
  return std::string(payload.substr(value_start, value_end - value_start));
}

std::optional<std::string_view> ExtractStructuredField(
    std::string_view payload,
    std::string_view key,
    char open_char,
    char close_char) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  std::size_t cursor = SkipWhitespace(payload, start + needle.size());
  if (cursor >= payload.size() || payload[cursor] != open_char) {
    return std::nullopt;
  }
  const std::size_t value_start = cursor + 1;
  int depth = 0;
  for (; cursor < payload.size(); ++cursor) {
    if (payload[cursor] == open_char) {
      ++depth;
    } else if (payload[cursor] == close_char) {
      --depth;
      if (depth == 0) {
        return payload.substr(value_start, cursor - value_start);
      }
    }
  }
  return std::nullopt;
}

std::optional<std::string_view> ExtractObjectField(std::string_view payload, std::string_view key) {
  return ExtractStructuredField(payload, key, '{', '}');
}

std::optional<std::string_view> ExtractArrayField(std::string_view payload, std::string_view key) {
  return ExtractStructuredField(payload, key, '[', ']');
}

std::optional<std::uint32_t> ExtractUintField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(parsed);
}

std::optional<float> ExtractFloatField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const float parsed = std::strtof(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<std::array<float, 3>> ExtractFloat3Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }
  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 3> out{};
  if (!(in >> out[0] >> out[1] >> out[2])) {
    return std::nullopt;
  }
  return out;
}

std::optional<std::array<float, 4>> ExtractFloat4Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }
  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 4> out{};
  if (!(in >> out[0] >> out[1] >> out[2] >> out[3])) {
    return std::nullopt;
  }
  return out;
}

std::optional<int> ExtractIntField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const long parsed = std::strtol(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return static_cast<int>(parsed);
}

std::optional<double> ExtractDoubleField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const double parsed = std::strtod(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<bool> ExtractBoolField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t value_start = SkipWhitespace(payload, start + needle.size());
  if (payload.substr(value_start, 4) == "true") {
    return true;
  }
  if (payload.substr(value_start, 5) == "false") {
    return false;
  }
  return std::nullopt;
}

std::optional<std::vector<float>> ExtractFloatArrayField(std::string_view payload, std::string_view key) {
  const auto array_value = ExtractArrayField(payload, key);
  if (!array_value.has_value()) {
    return std::nullopt;
  }

  std::string values(*array_value);
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::vector<float> out;
  float value = 0.0f;
  while (in >> value) {
    out.push_back(value);
  }
  if (out.empty()) {
    return std::nullopt;
  }
  return out;
}

}  // namespace visualiser::parsing
