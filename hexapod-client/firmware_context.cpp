#include "firmware_context.hpp"

namespace {
FirmwareContext g_firmware{};
} // namespace

FirmwareContext& firmware()
{
  return g_firmware;
}
