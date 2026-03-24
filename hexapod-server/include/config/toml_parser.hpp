#ifndef TOML_PARSER_HPP
#define TOML_PARSER_HPP

#include <memory>
#include <string>

#include <toml.hpp>

#include "hexapod-server.hpp"
#include "logger.hpp"

class TomlParser
{
public:
  explicit TomlParser(std::shared_ptr<logging::AsyncLogger> logger = nullptr);

  bool parse(const std::string& filename, ParsedToml& out) const;

private:
  std::shared_ptr<logging::AsyncLogger> logger_;
  bool parseSchemaHeaderSection(const toml::value& root) const;
};

#endif // TOML_PARSER_HPP
