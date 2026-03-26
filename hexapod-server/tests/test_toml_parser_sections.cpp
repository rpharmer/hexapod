#include "control_config.hpp"
#include "geometry_config.hpp"
#include "hexapod-server.hpp"
#include "toml_parser.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "logger.hpp"
namespace {

int g_failures = 0;

class CollectingSink final : public logging::LogSink {
public:
  void Write(logging::LogLevel,
             std::string_view,
             std::string_view message,
             const logging::SourceLocation&) override
  {
    messages.emplace_back(message);
  }

  std::vector<std::string> messages;
};


std::shared_ptr<logging::AsyncLogger> makeTestLogger()
{
  auto logger = std::make_shared<logging::AsyncLogger>("test-toml", logging::LogLevel::Trace, 1024);
  logger->AddSink(std::make_shared<CollectingSink>());
  return logger;
}
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

bool containsMessage(const std::vector<std::string>& messages, const std::string& needle)
{
  for (const auto& message : messages) {
    if (message.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool testMalformedRuntimeModeFails()
{
  const auto sink = std::make_shared<CollectingSink>();
  auto logger = std::make_shared<logging::AsyncLogger>("test-toml", logging::LogLevel::Trace, 1024);
  logger->AddSink(sink);
  std::string cfg = readText(configPath("config.txt"));
  const std::string needle = "Runtime.Mode = \"serial\"";
  if (!expect(replaceOnce(cfg, needle, "Runtime.Mode = \"invalid\""),
              "baseline config missing Runtime.Mode entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(logger);
  const bool parse_failed = expect(!parser.parse(writeTemp("hexapod_bad_runtime.toml", cfg), parsed),
                                   "invalid Runtime.Mode should fail parser");
  logger->Flush();
  const bool logged = expect(containsMessage(sink->messages, "Runtime.Mode"),
                             "runtime mode parse failure should still emit an error log");
  const bool has_diagnostic =
      expect(!parsed.diagnostics.empty(), "invalid Runtime.Mode should produce diagnostics");
  const bool diagnostic_fields = expect(
      !parsed.diagnostics.empty() && parsed.diagnostics.front().section == "runtime" &&
          parsed.diagnostics.front().key == "Runtime.Mode" &&
          parsed.diagnostics.front().errorCode == "invalid_enum",
      "invalid Runtime.Mode diagnostic should include section/key/error code");
  logger->Stop();
  return parse_failed && logged && has_diagnostic && diagnostic_fields;
}

bool testMalformedCalibrationKeyFails()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string needle = "[\"R31\", 1031, 2088]";
  if (!expect(replaceOnce(cfg, needle, "[\"R99\", 1031, 2088]"),
              "baseline config missing R31 calibration entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  return expect(!parser.parse(writeTemp("hexapod_bad_cal_key.toml", cfg), parsed),
                "unexpected MotorCalibrations key should fail parser");
}

bool testDuplicateCalibrationKeyFails()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string needle = "[\"L13\", 1035, 2027]";
  if (!expect(replaceOnce(cfg, needle, "[\"R31\", 1035, 2027]"),
              "baseline config missing L13 calibration entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  return expect(!parser.parse(writeTemp("hexapod_dup_cal_key.toml", cfg), parsed),
                "duplicate MotorCalibrations key should fail parser");
}

bool testOutOfRangeTuningFallsBackToDefaults()
{
  std::string cfg = readText(configPath("config.txt"));
  const std::string from = "BusLoopPeriodUs = 2000";
  if (!expect(replaceOnce(cfg, from, "BusLoopPeriodUs = 100"),
              "baseline config missing BusLoopPeriodUs entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_bad_tuning_range.toml", cfg), parsed),
              "out-of-range tuning should still parse with defaults")) {
    return false;
  }

  return expect(parsed.busLoopPeriodUs == control_config::kDefaultBusLoopPeriodUs,
                "BusLoopPeriodUs should fallback to default when out of range");
}

bool testOutOfRangeRuntimeFallsBackWithDiagnostic()
{
  std::string cfg = readText(configPath("config.txt"));
  if (!expect(replaceOnce(cfg, "Runtime.Sim.InitialVoltageV = 12.0",
                          "Runtime.Sim.InitialVoltageV = 100.0"),
              "baseline config missing Runtime.Sim.InitialVoltageV entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_bad_runtime_sim_voltage.toml", cfg), parsed),
              "out-of-range runtime scalar should parse with fallback")) {
    return false;
  }

  if (!expect(near(parsed.simInitialVoltageV, 12.0),
              "Runtime.Sim.InitialVoltageV should fallback to default")) {
    return false;
  }

  for (const auto& diagnostic : parsed.diagnostics) {
    if (diagnostic.section == "runtime" && diagnostic.key == "Runtime.Sim.InitialVoltageV" &&
        diagnostic.errorCode == "out_of_range") {
      return true;
    }
  }
  return expect(false, "runtime fallback should emit out_of_range diagnostic");
}

bool testTelemetryRuntimeFieldsParseAndFallback()
{
  std::string cfg = readText(configPath("config.txt"));
  if (!expect(replaceOnce(cfg, "Runtime.Telemetry.Enable = false", "Runtime.Telemetry.Enable = true"),
              "baseline config missing Runtime.Telemetry.Enable entry") ||
      !expect(replaceOnce(cfg, "Runtime.Telemetry.Host = \"127.0.0.1\"",
                          "Runtime.Telemetry.Host = \"\""),
              "baseline config missing Runtime.Telemetry.Host entry") ||
      !expect(replaceOnce(cfg, "Runtime.Telemetry.Port = 9870", "Runtime.Telemetry.Port = 99999"),
              "baseline config missing Runtime.Telemetry.Port entry") ||
      !expect(replaceOnce(cfg, "Runtime.Telemetry.PublishRateHz = 30.0",
                          "Runtime.Telemetry.PublishRateHz = 0.01"),
              "baseline config missing Runtime.Telemetry.PublishRateHz entry") ||
      !expect(replaceOnce(cfg, "Runtime.Telemetry.GeometryResendIntervalSec = 1.0",
                          "Runtime.Telemetry.GeometryResendIntervalSec = 0.01"),
              "baseline config missing Runtime.Telemetry.GeometryResendIntervalSec entry")) {
    return false;
  }

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_runtime_telemetry_fallback.toml", cfg), parsed),
              "telemetry runtime values should parse with fallback on invalid scalars")) {
    return false;
  }

  if (!expect(parsed.telemetryEnabled, "telemetry enable should parse explicit true") ||
      !expect(parsed.telemetryHost == "127.0.0.1", "empty telemetry host should fallback to default") ||
      !expect(parsed.telemetryPort == 9870, "out-of-range telemetry port should fallback to default") ||
      !expect(near(parsed.telemetryPublishRateHz, 30.0),
              "out-of-range telemetry publish rate should fallback to default") ||
      !expect(near(parsed.telemetryGeometryResendIntervalSec, 1.0),
              "out-of-range telemetry resend interval should fallback to default")) {
    return false;
  }

  bool saw_host = false;
  bool saw_port = false;
  for (const auto& diagnostic : parsed.diagnostics) {
    if (diagnostic.section == "runtime" && diagnostic.key == "Runtime.Telemetry.Host" &&
        diagnostic.errorCode == "empty_value") {
      saw_host = true;
    }
    if (diagnostic.section == "runtime" && diagnostic.key == "Runtime.Telemetry.Port" &&
        diagnostic.errorCode == "out_of_range") {
      saw_port = true;
    }
  }
  return expect(saw_host, "telemetry host fallback should emit empty_value diagnostic") &&
         expect(saw_port, "telemetry port fallback should emit out_of_range diagnostic");
}

bool testImuRuntimeReadGateParsesAndDefaults()
{
  ParsedToml defaults{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(configPath("config.txt"), defaults), "config.txt should parse for imu defaults")) {
    return false;
  }
  if (!expect(!defaults.imuEnableReads, "Runtime.Imu.EnableReads should default to false")) {
    return false;
  }

  std::string cfg = readText(configPath("config.txt"));
  if (!expect(replaceOnce(cfg, "Runtime.Imu.EnableReads = false", "Runtime.Imu.EnableReads = true"),
              "baseline config missing Runtime.Imu.EnableReads entry")) {
    return false;
  }

  ParsedToml parsed{};
  if (!expect(parser.parse(writeTemp("hexapod_runtime_imu_enabled.toml", cfg), parsed),
              "Runtime.Imu.EnableReads=true should parse")) {
    return false;
  }
  return expect(parsed.imuEnableReads, "Runtime.Imu.EnableReads should parse explicit true");
}

bool testCalibrationNormalizationStableForShuffledTable()
{
  const std::string ordered =
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

  const std::string shuffled =
      "title = \"Hexapod Config File\"\n"
      "Schema = \"hexapod.server.config\"\n"
      "SchemaVersion = 1\n"
      "Runtime.Mode = \"serial\"\n"
      "SerialDevice = '/dev/ttyACM0'\n"
      "BaudRate = 115200\n"
      "Timeout_ms = 100\n"
      "MotorCalibrations = [\n"
      "[\"L12\", 951, 1978], [\"L13\", 1035, 2027], [\"R33\", 958, 1990],\n"
      "[\"R32\", 1003, 2016], [\"R31\", 1031, 2088], [\"L31\", 941, 2022],\n"
      "[\"L32\", 986, 2039], [\"L33\", 958, 1988], [\"R23\", 1057, 2090],\n"
      "[\"R22\", 976, 2019], [\"R21\", 1007, 2048], [\"L21\", 993, 2015],\n"
      "[\"L22\", 1011, 2013], [\"L23\", 956, 2000], [\"R11\", 1040, 2055],\n"
      "[\"R12\", 983, 2057], [\"R13\", 959, 1995], [\"L11\", 1031, 1998]\n"
      "]\n";

  ParsedToml baseline{};
  ParsedToml shuffled_parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_ordered_cals.toml", ordered), baseline),
              "ordered MotorCalibrations should parse") ||
      !expect(parser.parse(writeTemp("hexapod_shuffled_cals.toml", shuffled), shuffled_parsed),
              "shuffled MotorCalibrations should parse")) {
    return false;
  }

  return expect(baseline.minMaxPulses == shuffled_parsed.minMaxPulses,
                "normalized MotorCalibrations output should be stable across row order");
}

bool testSimModeTransportOptional()
{
  const std::string minimal_sim =
      "title = \"Hexapod Config File\"\n"
      "Schema = \"hexapod.server.config\"\n"
      "SchemaVersion = 1\n"
      "Runtime.Mode = \"sim\"\n"
      "MotorCalibrations = [\n"
      "[\"R31\", 1031, 2088], [\"R32\", 1003, 2016], [\"R33\", 958, 1990],\n"
      "[\"L31\", 941, 2022], [\"L32\", 986, 2039], [\"L33\", 958, 1988],\n"
      "[\"R21\", 1007, 2048], [\"R22\", 976, 2019], [\"R23\", 1057, 2090],\n"
      "[\"L21\", 993, 2015], [\"L22\", 1011, 2013], [\"L23\", 956, 2000],\n"
      "[\"R11\", 1040, 2055], [\"R12\", 983, 2057], [\"R13\", 959, 1995],\n"
      "[\"L11\", 1031, 1998], [\"L12\", 951, 1978], [\"L13\", 1035, 2027]\n"
      "]\n";

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_sim_minimal.toml", minimal_sim), parsed),
              "sim config should parse without transport section")) {
    return false;
  }
  return expect(parsed.runtimeMode == "sim", "runtime mode should be sim") &&
         expect(parsed.serialDevice == "/dev/ttyACM0", "serial fallback should remain default in sim mode");
}

bool testBaselineConfigParity()
{
  ParsedToml baseline_serial{};
  ParsedToml baseline_sim{};
  TomlParser parser(makeTestLogger());
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
                "default timeout parity should match config.sim.txt baseline") &&
         expect(minimal_parsed.logToFile == baseline_serial.logToFile,
                "default file logging toggle should match baseline config") &&
         expect(minimal_parsed.logFilePath == baseline_serial.logFilePath,
                "default log file path should match baseline config");
}



bool testRuntimeLoggingOverridesParse()
{
  const std::string cfg =
      "title = \"Hexapod Config File\"\n"
      "Schema = \"hexapod.server.config\"\n"
      "SchemaVersion = 1\n"
      "Runtime.Mode = \"sim\"\n"
      "Runtime.Log.FilePath = \"ci.log\"\n"
      "Runtime.Log.EnableFile = false\n"
      "MotorCalibrations = [\n"
      "[\"R31\", 1031, 2088], [\"R32\", 1003, 2016], [\"R33\", 958, 1990],\n"
      "[\"L31\", 941, 2022], [\"L32\", 986, 2039], [\"L33\", 958, 1988],\n"
      "[\"R21\", 1007, 2048], [\"R22\", 976, 2019], [\"R23\", 1057, 2090],\n"
      "[\"L21\", 993, 2015], [\"L22\", 1011, 2013], [\"L23\", 956, 2000],\n"
      "[\"R11\", 1040, 2055], [\"R12\", 983, 2057], [\"R13\", 959, 1995],\n"
      "[\"L11\", 1031, 1998], [\"L12\", 951, 1978], [\"L13\", 1035, 2027]\n"
      "]\n";

  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(writeTemp("hexapod_runtime_log_overrides.toml", cfg), parsed),
              "runtime logging overrides should parse")) {
    return false;
  }

  return expect(parsed.logFilePath == "ci.log", "Runtime.Log.FilePath should be parsed") &&
         expect(!parsed.logToFile, "Runtime.Log.EnableFile should be parsed");
}

bool testGeometryDynamicsLoadedFromParsedConfig()
{
  ParsedToml parsed{};
  TomlParser parser(makeTestLogger());
  if (!expect(parser.parse(configPath("config.txt"), parsed), "config.txt should parse")) {
    return false;
  }
  if (!expect(parsed.servoDynamicsPositiveTauS.size() == kNumLegs,
              "servo dynamics tau should be populated per leg")) {
    return false;
  }
  if (!expect(parsed.servoDynamicsNegativeVmaxRadps.size() == kNumLegs,
              "servo dynamics vmax should be populated per leg")) {
    return false;
  }
  parsed.servoDynamicsPositiveTauS[0] = Vec3{0.11, 0.12, 0.13};
  parsed.servoDynamicsNegativeVmaxRadps[0] = Vec3{6.1, 6.2, 6.3};

  geometry_config::loadFromParsedToml(parsed);
  const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();

  return expect(near(geometry.legGeometry[0].servoDynamics[COXA].positive_direction.tau_s, 0.11),
                "coxa positive tau should load from parsed config") &&
         expect(near(geometry.legGeometry[0].servoDynamics[FEMUR].positive_direction.tau_s, 0.12),
                "femur positive tau should load from parsed config") &&
         expect(near(geometry.legGeometry[0].servoDynamics[TIBIA].negative_direction.vmax_radps,
                     6.3),
                "tibia negative vmax should load from parsed config");
}

bool testGeometryCanBeWrittenBackToParsedConfig()
{
  HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
  geometry.legGeometry[0].servoDynamics[COXA].positive_direction.tau_s = 0.14;
  geometry.legGeometry[1].servoDynamics[FEMUR].negative_direction.vmax_radps = 5.5;

  ParsedToml out{};
  geometry_config::writeToParsedToml(out, geometry);

  return expect(near(out.servoDynamicsPositiveTauS[0].x, 0.14),
                "writeToParsedToml should export positive tau") &&
         expect(near(out.servoDynamicsNegativeVmaxRadps[1].y, 5.5),
                "writeToParsedToml should export negative vmax") &&
         expect(near(out.coxaLengthM, geometry.legGeometry[0].coxaLength.value),
                "writeToParsedToml should export geometry dimensions");
}

} // namespace

int main()
{
  testMalformedRuntimeModeFails();
  testMalformedCalibrationKeyFails();
  testDuplicateCalibrationKeyFails();
  testOutOfRangeTuningFallsBackToDefaults();
  testOutOfRangeRuntimeFallsBackWithDiagnostic();
  testTelemetryRuntimeFieldsParseAndFallback();
  testImuRuntimeReadGateParsesAndDefaults();
  testCalibrationNormalizationStableForShuffledTable();
  testSimModeTransportOptional();
  testBaselineConfigParity();
  testRuntimeLoggingOverridesParse();
  testGeometryDynamicsLoadedFromParsedConfig();
  testGeometryCanBeWrittenBackToParsedConfig();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
