#pragma once
#include <array>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <string>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace b_route {
namespace util {

int8_t nibble(char c);
char hexchar(int b, bool upper = false);

template<size_t N> bool hex2bin(std::string str, std::array<uint8_t, N> &out, size_t &out_len) {
  if ((str.length() & 1) == 1 || str.length() > N * 2) {
    return false;
  }
  size_t olen = str.length() / 2;
  for (int i = 0; i < olen; i++) {
    int8_t n1 = nibble(str[i * 2]);
    int8_t n2 = nibble(str[i * 2 + 1]);
    if (n1 < 0 || n2 < 0) {
      return false;
    }
    out[i] = uint8_t{static_cast<uint8_t>((n1 << 4) + n2)};
  }
  out_len = olen;
  return true;
}

std::string ltrim_sv(std::string &str);
std::string rtrim_sv(const std::string &str);
std::string trim_sv(std::string &str);

}  // namespace util
}  // namespace b_route

}  // namespace esphome
