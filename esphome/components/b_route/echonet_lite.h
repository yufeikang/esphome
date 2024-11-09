#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace b_route {
namespace echonet_lite {

constexpr int MAX_PROPERTIES = 3;
static constexpr uint8_t EHD1 = 0x10;
constexpr uint16_t UDP_PORT = 3610;

static constexpr uint8_t EH_D2_FORMAT1 = 0x81;
static constexpr uint8_t EH_D2_FORMAT2 = 0x82;

struct __attribute__((__packed__)) EOJ {
  uint8_t X1;
  uint8_t X2;
  uint8_t X3;
};

struct __attribute__((__packed__)) Property {
  uint8_t epc;
  uint8_t pdc;
  int offset;
};

struct __attribute__((__packed__)) Packet {
  uint8_t ehd1;
  uint8_t ehd2;
  uint16_t tid;
  EOJ seoj;
  EOJ deoj;
  uint8_t esv;
  uint8_t opc;
  Property properties[MAX_PROPERTIES];
};

struct __attribute__((__packed__)) IntegralPowerWithDateTime {
  uint16_t year;
  uint8_t mon;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint32_t value;
};

enum class ESV : uint8_t {
  GET_SNA = 0x52,
  GET = 0x62,
  GET_RES = 0x72,
  INF = 0x73,
};

class Codec {
 private:
  static constexpr uint16_t TID = 0;

 public:
  static void write_eoj(const EOJ &eoj, uint8_t *&dest, size_t max_size, size_t &written) {
    if (max_size >= written + 3) {
      *dest++ = uint8_t{eoj.X1};
      *dest++ = uint8_t{eoj.X2};
      *dest++ = uint8_t{eoj.X3};
    }
    written += 3;
  }
  static bool decode_packet(const uint8_t *data, size_t data_len, Packet &out);

  template<typename PropertyCodes, size_t N>
  static size_t encode_property_get(std::array<uint8_t, N> &out, const EOJ &seoj, const EOJ &deoj,
                                    const PropertyCodes &property_codes) {
    size_t written = 0;
    uint8_t *dest = std::begin(out);
    if (N >= 2) {
      *dest++ = uint8_t{EHD1};
      *dest++ = uint8_t{EH_D2_FORMAT1};
    }
    written += 2;
    // TID
    if (N >= written + 2) {
      *dest++ = uint8_t{TID >> 8};
      *dest++ = uint8_t{TID & 0xff};
    }
    written += 2;
    // SEOJ, DEOJ
    write_eoj(seoj, dest, N, written);
    write_eoj(deoj, dest, N, written);
    // ESV
    if (N >= written + 1) {
      *dest++ = static_cast<uint8_t>(ESV::GET);
    }
    written += 1;
    int cnt = 0;
    uint8_t *opc_p = dest++;
    written += 1;
    for (uint8_t prop : property_codes) {
      cnt++;
      if (N >= written + 2) {
        *dest++ = uint8_t{prop};
        *dest++ = uint8_t{0};
      }
      written += 2;
    }
    if (cnt <= 255) {
      *opc_p = static_cast<uint8_t>(cnt);
    }
    return written;
  }
  static int32_t get_signed_long(const uint8_t *buffer) { return static_cast<int32_t>(get_unsigned_long(buffer)); }
  static uint32_t get_unsigned_long(const uint8_t *buffer) {
    return (static_cast<uint8_t>(buffer[0]) << 24) + (static_cast<uint8_t>(buffer[1]) << 16) +
           (static_cast<uint8_t>(buffer[2]) << 8) + static_cast<uint8_t>(buffer[3]);
  }
  static uint16_t get_unsigned_short(const uint8_t *buffer) {
    return (static_cast<uint8_t>(buffer[0]) << 8) + static_cast<uint8_t>(buffer[1]);
  }
};

namespace props {
namespace lowv_smart_meter {

constexpr uint8_t ENERGY_COEFF = 0xD3;
constexpr uint8_t ENERGY_UNIT = 0xE1;
constexpr uint8_t INTEGRAL_ENERGY_FWD = 0xE0;
constexpr uint8_t MOMENTARY_POWER = 0xE7;
constexpr uint8_t SCHEDULED_INTEGRAL_ENERGY_FWD = 0xEA;

}  // namespace lowv_smart_meter
}  // namespace props
}  // namespace echonet_lite
}  // namespace b_route
}  // namespace esphome
