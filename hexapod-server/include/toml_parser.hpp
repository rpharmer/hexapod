#ifndef TOML_PARSER_HPP
#define TOML_PARSER_HPP

#include <string>

#include <toml.hpp>

#include "hexapod-server.hpp"

class TomlParser
{
public:
  bool parse(const std::string& filename, ParsedToml& out) const;

private:
  bool validateSchemaHeader(const toml::value& root) const;
  bool parseRuntimeConfig(const toml::value& root, ParsedToml& out) const;
  bool parseTransportConfig(const toml::value& root, ParsedToml& out, bool required) const;
  bool parseCalibrationConfig(const toml::value& root, ParsedToml& out) const;
  void parseTuningConfig(const toml::value& root, ParsedToml& out) const;
  void parseGeometryConfig(const toml::value& root, ParsedToml& out) const;
};

#endif // TOML_PARSER_HPP
