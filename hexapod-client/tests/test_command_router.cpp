#include "command_router.hpp"
#include "hexapod-common.hpp"

#include <cstdlib>
#include <iostream>
#include <type_traits>

struct FirmwareContext {
  int called_handler{0};
  int called_nack{0};
  uint16_t last_seq{0};
  std::vector<uint8_t> last_payload{};
};

namespace {

bool expect(bool condition, const char* message)
{
  if(!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}

void handlerA(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  ctx.called_handler++;
  ctx.last_seq = seq;
  ctx.last_payload = payload;
}

void nackResponder(FirmwareContext& ctx, uint16_t seq)
{
  ctx.called_nack++;
  ctx.last_seq = seq;
}

bool test_dispatches_matching_route()
{
  FirmwareContext ctx{};
  const CommandRoute routes[] = {
      {CommandCode::HELLO, {PayloadPolicyType::ExactBytes, 2}, handlerA},
  };
  const DecodedPacket packet{42, as_u8(CommandCode::HELLO), {0xAB, 0xCD}};

  const bool handled = dispatchCommand(ctx, CommandCode::HELLO, packet.seq, packet.payload, routes, 1, nackResponder);
  return expect(handled, "expected packet to be handled") &&
         expect(ctx.called_handler == 1, "expected route handler call") &&
         expect(ctx.called_nack == 0, "expected no nack responder call");
}

bool test_rejects_invalid_payload_length()
{
  FirmwareContext ctx{};
  const CommandRoute routes[] = {
      {CommandCode::HELLO, {PayloadPolicyType::ExactBytes, 2}, handlerA},
  };
  const DecodedPacket packet{7, as_u8(CommandCode::HELLO), {0xAB}};

  const bool handled = dispatchCommand(ctx, CommandCode::HELLO, packet.seq, packet.payload, routes, 1, nackResponder);
  return expect(handled, "expected invalid payload route to be consumed") &&
         expect(ctx.called_handler == 0, "expected no handler call") &&
         expect(ctx.called_nack == 1, "expected nack responder call");
}

bool test_returns_false_for_unknown_command()
{
  FirmwareContext ctx{};
  const CommandRoute routes[] = {
      {CommandCode::HELLO, {PayloadPolicyType::ExactBytes, 2}, handlerA},
  };
  const DecodedPacket packet{9, as_u8(CommandCode::KILL), {0xAB, 0xCD}};

  const bool handled = dispatchCommand(ctx, CommandCode::KILL, packet.seq, packet.payload, routes, 1, nackResponder);
  return expect(!handled, "expected unknown command to return false") &&
         expect(ctx.called_handler == 0, "expected no handler call") &&
         expect(ctx.called_nack == 0, "expected no nack for unknown command");
}

bool test_dispatch_signature_requires_typed_command_code()
{
  constexpr bool accepts_typed =
      std::is_invocable_r_v<bool,
                            decltype(dispatchCommand),
                            FirmwareContext&,
                            CommandCode,
                            uint16_t,
                            const std::vector<uint8_t>&,
                            const CommandRoute*,
                            std::size_t,
                            PayloadLengthErrorResponder>;
  constexpr bool accepts_raw =
      std::is_invocable_r_v<bool,
                            decltype(dispatchCommand),
                            FirmwareContext&,
                            const DecodedPacket&,
                            const CommandRoute*,
                            std::size_t,
                            PayloadLengthErrorResponder>;
  static_assert(accepts_typed, "dispatchCommand should accept typed CommandCode");
  static_assert(!accepts_raw, "dispatchCommand should not accept raw DecodedPacket command IDs");
  return true;
}


bool test_find_route_signature_requires_typed_command_code()
{
  constexpr bool accepts_typed =
      std::is_invocable_r_v<const CommandRoute*,
                            decltype(findRoute),
                            const CommandRoute*,
                            std::size_t,
                            CommandCode>;
  constexpr bool accepts_raw =
      std::is_invocable_r_v<const CommandRoute*,
                            decltype(findRoute),
                            const CommandRoute*,
                            std::size_t,
                            uint8_t>;
  static_assert(accepts_typed, "findRoute should accept typed CommandCode");
  static_assert(!accepts_raw, "findRoute should not accept raw command IDs");
  return true;
}

} // namespace

int main()
{
  if(!test_dispatches_matching_route())
    return EXIT_FAILURE;
  if(!test_rejects_invalid_payload_length())
    return EXIT_FAILURE;
  if(!test_returns_false_for_unknown_command())
    return EXIT_FAILURE;
  if(!test_dispatch_signature_requires_typed_command_code())
    return EXIT_FAILURE;
  if(!test_find_route_signature_requires_typed_command_code())
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
