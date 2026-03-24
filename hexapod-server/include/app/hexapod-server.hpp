// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"
#include "types.hpp"
#include "config_validation.hpp"


struct ParsedToml
{
  std::string runtimeMode{"serial"};

  std::string serialDevice{"/dev/ttyACM0"};
  int baudRate{115200};
  int timeout{100};
  std::vector<float> minMaxPulses{};

  double simInitialVoltageV{12.0};
  double simInitialCurrentA{1.0};
  double simResponseRateHz{50.0};
  bool simDropBus{false};
  bool simLowVoltage{false};
  bool simHighCurrent{false};

  std::string logFilePath{"app.log"};
  bool logToFile{true};
  bool telemetryEnabled{false};
  std::string telemetryHost{"127.0.0.1"};
  int telemetryPort{9870};
  double telemetryPublishRateHz{30.0};
  double telemetryGeometryResendIntervalSec{1.0};

  bool telemetryEnabled{false};
  std::string telemetryUdpHost{"127.0.0.1"};
  int telemetryUdpPort{9870};
  int telemetryPublishPeriodMs{50};
  int telemetryGeometryRefreshPeriodMs{2000};

  int busLoopPeriodUs{2000};
  int estimatorLoopPeriodUs{2000};
  int controlLoopPeriodUs{4000};
  int safetyLoopPeriodUs{2000};
  int diagnosticsPeriodMs{500};
  int commandRefreshPeriodMs{100};
  int standSettlingDelayMs{2000};
  double maxTiltRad{0.70};
  uint64_t commandTimeoutUs{300000};
  uint64_t estimatorMaxAgeUs{300000};
  uint64_t intentMaxAgeUs{300000};
  bool estimatorRequireTimestamp{true};
  bool estimatorRequireSampleId{true};
  bool estimatorRequireMonotonicSampleId{true};
  bool intentRequireTimestamp{true};
  bool intentRequireSampleId{true};
  bool intentRequireMonotonicSampleId{true};
  double fallbackSpeedMag{0.01};
  double minBusVoltageV{10.5};
  double maxBusCurrentA{25.0};
  int minFootContacts{0};
  int maxFootContacts{kNumLegs};

  double coxaLengthM{0.043};
  double femurLengthM{0.060};
  double tibiaLengthM{0.104};
  std::vector<double> mountAnglesDeg{};
  std::vector<Vec3> coxaOffsetsM{};
  double coxaAttachDeg{0.0};
  std::vector<double> femurAttachDeg{};
  std::vector<double> tibiaAttachDeg{};
  std::vector<double> sideSign{};
  double bodyToBottomM{0.040};
  std::vector<Vec3> servoDynamicsPositiveTauS{};
  std::vector<Vec3> servoDynamicsPositiveVmaxRadps{};
  std::vector<Vec3> servoDynamicsNegativeTauS{};
  std::vector<Vec3> servoDynamicsNegativeVmaxRadps{};
  std::vector<config_validation::ConfigDiagnostic> diagnostics{};
};

bool tomlParser(std::string filename,  ParsedToml& out);

#endif  // #ifndef HEXAPOD_SERVER_HPP
