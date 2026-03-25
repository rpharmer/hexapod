#ifndef COMMAND_DISPATCH_INTERNAL_HPP
#define COMMAND_DISPATCH_INTERNAL_HPP

#include "framing.hpp"

#include <cstdint>

struct FirmwareContext;

bool handleWaitingForHostPacket(FirmwareContext& ctx, const DecodedPacket& packet);
bool handleActivePacket(FirmwareContext& ctx, const DecodedPacket& packet);
void enforceHostLivenessTimeout(FirmwareContext& ctx, int64_t now_us, int64_t& last_host_activity_us);

#endif
