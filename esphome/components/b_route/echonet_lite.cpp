#include "echonet_lite.h"
#include <cstddef>

namespace esphome {
namespace b_route {
namespace echonet_lite {

bool Codec::decode_packet(const uint8_t *data, size_t data_len, Packet &out) {
  constexpr const size_t p_min = 12;
  static_assert(offsetof(Packet, properties) == p_min, "");
  if (data_len < 12) {
    return false;
  }
  std::copy(data, data + p_min, reinterpret_cast<uint8_t *>(&out));
  if (out.ehd1 != EHD1 || out.ehd2 != EH_D2_FORMAT1) {
    // currently EHD2_Format2 is not supported
    return false;
  }
  out.tid = (static_cast<uint8_t>(data[2]) << 8) + static_cast<uint8_t>(data[3]);
  for (int pos = p_min, n = 0; n < out.opc; n++) {
    if (data_len < pos + 2) {
      return false;
    }
    Property &prop = out.properties[n];
    prop.epc = static_cast<uint8_t>(data[pos++]);
    prop.pdc = static_cast<uint8_t>(data[pos++]);
    prop.offset = pos;
    if (data_len < pos + prop.pdc) {
      return false;
    }
    pos += prop.pdc;
  }
  return true;
}

}  // namespace echonet_lite
}  // namespace b_route
}  // namespace esphome
