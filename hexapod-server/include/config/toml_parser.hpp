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
};

#endif // TOML_PARSER_HPP
