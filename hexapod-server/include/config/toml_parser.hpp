#ifndef TOML_PARSER_HPP
#define TOML_PARSER_HPP

#include <memory>
#include <string>
#include <vector>

#include <toml.hpp>

#include "config_validation.hpp"
#include "hexapod-server.hpp"
#include "logger.hpp"

class TomlParser
{
public:
  explicit TomlParser(std::shared_ptr<logging::AsyncLogger> logger = nullptr);

  bool parse(const std::string& filename, ParsedToml& out) const;

private:
  std::shared_ptr<logging::AsyncLogger> logger_;
  bool parseSchemaHeaderSection(
      const toml::value& root,
      std::vector<config_validation::ConfigDiagnostic>* diagnostics) const;
};

#endif // TOML_PARSER_HPP
