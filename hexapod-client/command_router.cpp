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

bool dispatchCommand(FirmwareContext& ctx, CommandCode cmd, uint16_t seq, const std::vector<uint8_t>& payload,
                     const CommandRoute* routes, std::size_t routeCount,
                     PayloadLengthErrorResponder onPayloadLengthError)
{
  const CommandRoute* route = findRoute(routes, routeCount, cmd);
  if(route == nullptr)
    return false;

  if(!route->payloadPolicy.accepts(payload.size()))
  {
    onPayloadLengthError(ctx, seq);
    return true;
  }

  route->handler(ctx, seq, payload);
  return true;
}
