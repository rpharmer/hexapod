#pragma once

#include "framing.hpp"
#include "hexapod-common.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct FirmwareContext;

enum class PayloadPolicyType : uint8_t
{
  Any,
  ExactBytes
};

struct PayloadPolicy
{
  PayloadPolicyType type{PayloadPolicyType::Any};
  std::size_t expectedBytes{0};

  bool accepts(std::size_t payloadBytes) const
  {
    if(type == PayloadPolicyType::Any)
      return true;

    return payloadBytes == expectedBytes;
  }
};

using RoutedHandler = void (*)(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload);
using PayloadLengthErrorResponder = void (*)(FirmwareContext& ctx, uint16_t seq);

struct CommandRoute
{
  CommandCode cmd;
  PayloadPolicy payloadPolicy;
  RoutedHandler handler;
};

const CommandRoute* findRoute(const CommandRoute* routes, std::size_t routeCount, CommandCode cmd);
const CommandRoute* findRoute(const CommandRoute* routes, std::size_t routeCount, uint8_t cmd);

bool dispatchCommand(FirmwareContext& ctx, const DecodedPacket& packet,
                     const CommandRoute* routes, std::size_t routeCount,
                     PayloadLengthErrorResponder onPayloadLengthError);
