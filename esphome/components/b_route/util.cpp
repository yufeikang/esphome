#include "util.h"
#include <cstddef>
#include <cstring>
#include <utility>

namespace esphome {
namespace b_route {
namespace util {

int8_t nibble(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

char hexchar(int b, bool upper) {
  if (b >= 0 && b <= 9) {
    return '0' + b;
  }
  if (b >= 10 && b <= 15) {
    return (upper ? 'A' : 'a') + (b - 10);
  }
  return 0;
}

#if __GNUG__ <= 7
static const auto WHITESPACES = std::string(" \t\r\n");
#else
static constexpr auto WHITESPACES = std::string(" \t\r\n");
#endif

std::string ltrim_sv(const std::string &str) {
  auto pos = str.find_first_not_of(WHITESPACES);
  return pos == std::string::npos ? std::string{} : std::string{str}.substr(pos);
}

std::string rtrim_sv(const std::string &str) {
  auto pos = str.find_last_not_of(WHITESPACES);
  return pos == std::string::npos ? std::string{} : std::string{str}.substr(0, pos + 1);
}

std::string trim_sv(const std::string &str) { return rtrim_sv(ltrim_sv(str)); }

}  // namespace util
}  // namespace b_route
}  // namespace esphome
