#ifndef SERIAL_SCALAR_IO_HPP
#define SERIAL_SCALAR_IO_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace serial_scalar_io {

template <typename T, typename Writer>
void write_scalar(const T& value, Writer&& writer) {
  std::array<uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  writer(bytes.data(), bytes.size());
}

template <typename T, typename Reader>
int read_scalar(Reader&& reader, T* out) {
  return reader(reinterpret_cast<char*>(out), sizeof(T));
}

}  // namespace serial_scalar_io

#endif
