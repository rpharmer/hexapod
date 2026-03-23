#ifndef PAYLOAD_DECODE_HPP
#define PAYLOAD_DECODE_HPP

#include "framing.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace payload {

enum class DecodeStatus
{
  Ok,
  InvalidLength,
  Malformed
};

template <typename OnDecodeFailure>
bool decode_or_report(DecodeStatus status, OnDecodeFailure&& onDecodeFailure)
{
  if(status == DecodeStatus::Ok)
    return true;

  onDecodeFailure(status);
  return false;
}

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

template <typename T, typename Decoder, typename OnDecodeFailure>
bool expect_payload_or_report(const std::vector<uint8_t>& payload,
                              std::size_t expected_size,
                              T& out,
                              Decoder decoder,
                              OnDecodeFailure&& onDecodeFailure)
{
  return decode_or_report(expect_payload(payload, expected_size, out, decoder),
                          std::forward<OnDecodeFailure>(onDecodeFailure));
}

template <typename T, typename OnDecodeFailure>
bool decode_scalar_exact_or_report(const std::vector<uint8_t>& payload,
                                   T& out,
                                   OnDecodeFailure&& onDecodeFailure)
{
  const DecodeStatus status = decode_scalar_exact(payload, out)
      ? DecodeStatus::Ok
      : DecodeStatus::InvalidLength;
  return decode_or_report(status, std::forward<OnDecodeFailure>(onDecodeFailure));
}

template <typename A, typename B, typename OnDecodeFailure>
bool decode_two_scalars_exact_or_report(const std::vector<uint8_t>& payload,
                                        A& a,
                                        B& b,
                                        OnDecodeFailure&& onDecodeFailure)
{
  const DecodeStatus status = decode_two_scalars_exact(payload, a, b)
      ? DecodeStatus::Ok
      : DecodeStatus::InvalidLength;
  return decode_or_report(status, std::forward<OnDecodeFailure>(onDecodeFailure));
}

} // namespace payload

#endif
