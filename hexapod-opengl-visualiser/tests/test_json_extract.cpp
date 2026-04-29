#include "visualiser/parsing/json_extract.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main() {
  const std::string payload = R"({"name":"demo","f":1.25,"arr":[1,2,3],"b":true,"o":{"x":1}})";
  bool ok = true;
  ok = ok && expect(visualiser::parsing::ExtractStringField(payload, "name").value_or("") == "demo", "string");
  ok = ok && expect(visualiser::parsing::ExtractFloatField(payload, "f").has_value(), "float");
  ok = ok && expect(visualiser::parsing::ExtractBoolField(payload, "b").value_or(false), "bool");
  ok = ok && expect(visualiser::parsing::ExtractArrayField(payload, "arr").has_value(), "array");
  ok = ok && expect(visualiser::parsing::ExtractObjectField(payload, "o").has_value(), "object");
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
