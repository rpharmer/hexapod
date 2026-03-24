#include "command_router.hpp"

const CommandRoute* findRoute(const CommandRoute* routes, std::size_t routeCount, CommandCode cmd)
{
  for(std::size_t i = 0; i < routeCount; ++i)
  {
    if(routes[i].cmd == cmd)
      return &routes[i];
  }

  return nullptr;
}

const CommandRoute* findRoute(const CommandRoute* routes, std::size_t routeCount, uint8_t cmd)
{
  return findRoute(routes, routeCount, static_cast<CommandCode>(cmd));
}

bool dispatchCommand(FirmwareContext& ctx, const DecodedPacket& packet,
                     const CommandRoute* routes, std::size_t routeCount,
                     PayloadLengthErrorResponder onPayloadLengthError)
{
  const CommandRoute* route = findRoute(routes, routeCount, packet.cmd);
  if(route == nullptr)
    return false;

  if(!route->payloadPolicy.accepts(packet.payload.size()))
  {
    onPayloadLengthError(ctx, packet.seq);
    return true;
  }

  route->handler(ctx, packet.seq, packet.payload);
  return true;
}
