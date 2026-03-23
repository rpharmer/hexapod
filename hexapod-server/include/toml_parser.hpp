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
  bool parseSchemaHeaderSection(const toml::value& root) const;
  bool parseRuntimeSection(const toml::value& root, ParsedToml& out) const;
  bool parseTransportSection(const toml::value& root, ParsedToml& out, bool required) const;
  bool parseCalibrationsSection(const toml::value& root, ParsedToml& out) const;
  void parseTuningSection(const toml::value& root, ParsedToml& out) const;
  void parseGeometrySection(const toml::value& root, ParsedToml& out) const;
};

#endif // TOML_PARSER_HPP
