#include "control_config.hpp"
#include "hexapod-server.hpp"
#include "toml_parser.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int g_failures = 0;

bool expect(bool cond, const std::string& message)
{
  if (!cond) {
    std::cerr << "[FAIL] " << message << '\n';
    ++g_failures;
    return false;
  }
  return true;
}

bool near(double a, double b, double eps = 1e-9)
{
  return std::fabs(a - b) <= eps;
}

std::string readText(const std::string& path)
{
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string writeTemp(const std::string& name, const std::string& content)
{
  const std::string path = "/tmp/" + name;
  std::ofstream out(path);
  out << content;
  out.close();
  return path;
}


std::string configPath(const std::string& file)
{
  const auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
  return (base / file).string();
}

bool replaceOnce(std::string& text, const std::string& from, const std::string& to)
{
  const auto pos = text.find(from);
  if (pos == std::string::npos) {
    return false;
  }
  text.replace(pos, from.size(), to);
  return true;
}

bool testMalformedRuntimeModeFails()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string needle = "Runtime.Mode = \"serial\"";
  if (!expect(replaceOnce(cfg, needle, "Runtime.Mode = \"invalid\""), "baseline config missing Runtime.Mode entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser;
  return expect(!parser.parse(writeTemp("hexapod_bad_runtime.toml", cfg), parsed),
                "invalid Runtime.Mode should fail parser");
}

bool testMalformedCalibrationKeyFails()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string needle = "[\"R31\", 1031, 2088]";
  if (!expect(replaceOnce(cfg, needle, "[\"R99\", 1031, 2088]"), "baseline config missing R31 calibration entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser;
  return expect(!parser.parse(writeTemp("hexapod_bad_cal_key.toml", cfg), parsed),
                "unexpected MotorCalibrations key should fail parser");
}

bool testOutOfRangeTuningFallsBackToDefaults()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string from = "BusLoopPeriodUs = 2000";
  if (!expect(replaceOnce(cfg, from, "BusLoopPeriodUs = 100"), "baseline config missing BusLoopPeriodUs entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser;
  if (!expect(parser.parse(writeTemp("hexapod_bad_tuning_range.toml", cfg), parsed),
              "out-of-range tuning should still parse with defaults")) {
    return false;
  }

  return expect(parsed.busLoopPeriodUs == control_config::kDefaultBusLoopPeriodUs,
                "BusLoopPeriodUs should fallback to default when out of range");
}

bool testBaselineConfigParity()
{
  ParsedToml baseline_serial{};
  ParsedToml baseline_sim{};
  TomlParser parser;
  if (!expect(parser.parse(configPath("config.txt"), baseline_serial), "config.txt should parse") ||
      !expect(parser.parse(configPath("config.sim.txt"), baseline_sim), "config.sim.txt should parse")) {
    return false;
  }

  const std::string minimal =
      "title = \"Hexapod Config File\"\n"
      "Schema = \"hexapod.server.config\"\n"
      "SchemaVersion = 1\n"
      "Runtime.Mode = \"serial\"\n"
      "SerialDevice = '/dev/ttyACM0'\n"
      "BaudRate = 115200\n"
      "Timeout_ms = 100\n"
      "MotorCalibrations = [\n"
      "[\"R31\", 1031, 2088], [\"R32\", 1003, 2016], [\"R33\", 958, 1990],\n"
      "[\"L31\", 941, 2022], [\"L32\", 986, 2039], [\"L33\", 958, 1988],\n"
      "[\"R21\", 1007, 2048], [\"R22\", 976, 2019], [\"R23\", 1057, 2090],\n"
      "[\"L21\", 993, 2015], [\"L22\", 1011, 2013], [\"L23\", 956, 2000],\n"
      "[\"R11\", 1040, 2055], [\"R12\", 983, 2057], [\"R13\", 959, 1995],\n"
      "[\"L11\", 1031, 1998], [\"L12\", 951, 1978], [\"L13\", 1035, 2027]\n"
      "]\n";

  ParsedToml minimal_parsed{};
  if (!expect(parser.parse(writeTemp("hexapod_minimal_defaults.toml", minimal), minimal_parsed),
              "minimal config should parse")) {
    return false;
  }

  return expect(minimal_parsed.busLoopPeriodUs == baseline_serial.busLoopPeriodUs,
                "minimal config tuning defaults should match config.txt baseline") &&
         expect(near(minimal_parsed.coxaLengthM, baseline_serial.coxaLengthM),
                "minimal config geometry defaults should match config.txt baseline") &&
         expect(minimal_parsed.commandTimeoutUs == baseline_sim.commandTimeoutUs,
                "default timeout parity should match config.sim.txt baseline");
}

} // namespace

int main()
{
  testMalformedRuntimeModeFails();
  testMalformedCalibrationKeyFails();
  testOutOfRangeTuningFallsBackToDefaults();
  testBaselineConfigParity();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
