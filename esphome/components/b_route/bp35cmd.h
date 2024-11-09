#pragma once
#include <string>
#include <type_traits>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace b_route {
namespace cmd {
namespace arg {

char hexchar(uint8_t nibble);
int hexvalue(char c);
std::string nibble(uint8_t b);
std::string flag(bool b);
std::string mode(uint8_t mode);
std::string num8(uint8_t n);
std::string num16(uint16_t n);
std::string num32(uint32_t n);
std::string reg(uint8_t num);
std::string ipv6(uint8_t (&addr)[16]);
std::string mac(uint8_t (&addr)[8]);
inline std::string str(std::string s) { return s; }

template<typename T>
using is_char_iter = typename std::enable_if<std::is_same<typename std::iterator_traits<T>::value_type, char>::value,
                                             std::nullptr_t>::type;

template<typename T, is_char_iter<T> = nullptr> inline bool get_num8(T &cur, const T &end, uint8_t &out) {
  if (cur == end) {
    return false;
  }
  int v = hexvalue(*cur++);
  if (v < 0) {
    return false;
  }
  if (cur == end) {
    return false;
  }
  int v2 = hexvalue(*cur++);
  if (v2 < 0) {
    return false;
  }
  out = (v << 4) + v2;
  return true;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_num16(T &cur, const T &end, uint16_t &out) {
  uint8_t v1, v2;
  if (get_num8(cur, end, v1) && get_num8(cur, end, v2)) {
    out = (v1 << 8) + v2;
    return true;
  }
  return false;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_num32(T &cur, const T &end, uint32_t &out) {
  uint16_t v1, v2;
  if (get_num16(cur, end, v1) && get_num16(cur, end, v2)) {
    out = (v1 << 16) + v2;
    return true;
  }
  return false;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_mac(T &cur, const T &end, uint8_t (&addr)[8]) {
  uint8_t v;
  for (unsigned char &i : addr) {
    if (!get_num8(cur, end, v)) {
      return false;
    }
    i = v;
  }
  return true;
}

template<typename T, is_char_iter<T> = nullptr> inline bool skip_sep(T &cur, const T &end) {
  if (cur == end) {
    return false;
  }
  if (*cur == ' ') {
    cur++;
    return true;
  }
  return false;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_flag(T &cur, const T &end, bool &out) {
  if (cur == end) {
    return false;
  }
  int v = hexvalue(*cur++);
  if (v != 0 && v != 1) {
    return false;
  }
  out = static_cast<bool>(v);
  return true;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_mode(T &cur, const T &end, uint8_t &out) {
  if (cur == end) {
    return false;
  }
  int v = hexvalue(*cur++);
  if (v < 0) {
    return false;
  }
  out = static_cast<uint8_t>(v);
  return true;
}

template<typename T, is_char_iter<T> = nullptr> inline bool get_ipv6(T &cur, const T &end, uint8_t (&addr)[16]) {
  uint16_t v;
  for (auto i = 0; i < 8; i++) {
    if (!get_num16(cur, end, v)) {
      return false;
    }
    if (i != 7) {
      if (cur == end || *cur++ != ':') {
        return false;
      }
    }
    addr[i * 2] = v >> 8;
    addr[i * 2 + 1] = v & 0xff;
  }
  return true;
}
}  // namespace arg
}  // namespace cmd
}  // namespace b_route
}  // namespace esphome
