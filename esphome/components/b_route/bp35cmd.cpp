#include "bp35cmd.h"
#include <string>

namespace esphome {
namespace b_route {
namespace cmd {
namespace arg {

char hexchar(uint8_t nibble) { return nibble < 10 ? '0' + nibble : 'A' + nibble - 10; }

int hexvalue(char c) {
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

std::string nibble(uint8_t b) { return {hexchar(b)}; }

std::string flag(bool b) { return b ? "1" : "0"; }

std::string mode(uint8_t mode) { return nibble(mode & 0x0f); }

std::string num8(uint8_t n) { return nibble(n >> 4) + nibble(n & 0x0f); }

std::string num16(uint16_t n) { return num8(n >> 8) + num8(n & 0xff); }

std::string num32(uint32_t n) { return num16(n >> 16) + num16(n & 0xffff); }

std::string reg(uint8_t num) { return "S" + num8(num); }

std::string ipv6(uint8_t (&addr)[16]) {
  return num16((addr[0] << 8) + addr[1]) + ':' + num16((addr[2] << 8) + addr[3]) + ':' +
         num16((addr[4] << 8) + addr[5]) + ':' + num16((addr[6] << 8) + addr[7]) + ':' +
         num16((addr[8] << 8) + addr[9]) + ':' + num16((addr[10] << 8) + addr[11]) + ':' +
         num16((addr[12] << 8) + addr[13]) + ':' + num16((addr[14] << 8) + addr[15]);
}

std::string mac(uint8_t (&addr)[8]) {
  return num32((addr[0] << 24) + (addr[1] << 16) + (addr[2] << 8) + addr[3]) +
         num32((addr[4] << 24) + (addr[5] << 16) + (addr[6] << 8) + addr[7]);
}

}  // namespace arg
}  // namespace cmd
}  // namespace b_route
}  // namespace esphome
