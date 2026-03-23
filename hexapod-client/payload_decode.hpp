#ifndef PAYLOAD_DECODE_HPP
#define PAYLOAD_DECODE_HPP

#include "framing.hpp"

#include <cstddef>
#include <vector>

namespace payload {

enum class DecodeStatus
{
  Ok,
  InvalidLength,
  Malformed
};

template <typename T, typename Decoder>
DecodeStatus expect_payload(const std::vector<uint8_t>& payload,
                            std::size_t expected_size,
                            T& out,
                            Decoder decoder)
{
  if(payload.size() != expected_size)
    return DecodeStatus::InvalidLength;

  if(!decoder(payload, out))
    return DecodeStatus::Malformed;

  return DecodeStatus::Ok;
}

template <typename T>
bool decode_scalar_exact(const std::vector<uint8_t>& payload, T& out)
{
  std::size_t offset = 0;
  return read_scalar(payload, offset, out) && offset == payload.size();
}

template <typename A, typename B>
bool decode_two_scalars_exact(const std::vector<uint8_t>& payload, A& a, B& b)
{
  std::size_t offset = 0;
  return read_scalar(payload, offset, a) &&
         read_scalar(payload, offset, b) &&
         offset == payload.size();
}

} // namespace payload

#endif
