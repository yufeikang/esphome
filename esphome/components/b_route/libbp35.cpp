#include "libbp35.h"
#include <esphome/core/hal.h>
#include "bp35cmd.h"

namespace esphome {
namespace b_route::libbp35 {

using namespace b_route::cmd;
const char *event_num_str(uint8_t num) {
  switch (num) {
    case 0x01:
      return "rcvNS";
    case 0x02:
      return "rcvNA";
    case 0x05:
      return "recvECHO";
    case 0x1f:
      return "doneEDscan";
    case 0x20:
      return "recvBCN";
    case 0x21:
      return "sentUDP";
    case 0x22:
      return "doneAScan";
    case 0x24:
      return "failedPANAconn";
    case 0x25:
      return "donePANAconn";
    case 0x26:
      return "recvDISC";
    case 0x27:
      return "donePANAdisc";
    case 0x28:
      return "timeoutPANDdisc";
    case 0x29:
      return "expiredSession";
    case 0x32:
      return "limitRate";
    case 0x33:
      return "canceledLimit";
    default:
      return "UNKNOWN";
  }
}

const char *event_str(EventT ev) {
  switch (ev) {
    case EventT::ERROR:
      return "error";
    case EventT::EVENT:
      return "event";
    case EventT::NONE:
      return "none";
    case EventT::OK:
      return "ok";
    case EventT::PANDESC:
      return "pandesc";
    case EventT::RXUDP:
      return "rxudp";
    case EventT::VER:
      return "ver";
    default:
    case EventT::UNKNOWN:
      return "unknown";
  }
}

bool BP35::read_line(std::string &line, uint32_t timeout) {
  auto start = esphome::millis();
  bool read1 = false;
  while (esphome::millis() - start < timeout) {
    int c = stream_.read();
    if (c == '\n')
      continue;
    if (c < 0) {
      if (!read1) {
        return false;
      }
      esphome::delay(1);
      continue;
    }
    read1 = true;
    while (c >= 0 && c != '\r') {
      if (c != '\n') {
        line += static_cast<char>(c);
      }
      c = stream_.read();
    }
    if (c == '\r') {
      return !line.empty();
    }
  }
  return false;
}

bool BP35::parse_rxudp(const std::string &remain, RxudpT &out) {
  auto pos = std::cbegin(remain);
  auto end = std::cend(remain);
  if (!arg::get_ipv6(pos, end, out.sender) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_ipv6(pos, end, out.dest) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_num16(pos, end, out.rport) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_num16(pos, end, out.lport) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_mac(pos, end, out.sender_lla) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_flag(pos, end, out.secured) || !arg::skip_sep(pos, end)) {
    return false;
  }
  if (!arg::get_num16(pos, end, out.data_len) || !arg::skip_sep(pos, end)) {
    return false;
  }
  out.data_pos = std::distance(std::cbegin(remain), pos);

  return true;
}

EventT BP35::get_event(uint32_t timeout, EventParamsT &params) {
  params.clear();
  std::string line;
  for (auto remain = timeout; remain > 0;) {
    line.clear();
    auto started = esphome::millis();
    if (!read_line(line, remain)) {
      return EventT::NONE;
    }
    if (line.rfind("SK", 0) == 0) {
      auto elapsed = esphome::millis() - started;
      if (remain > elapsed) {
        remain -= elapsed;
      } else {
        return EventT::NONE;
      }
      continue;
    }
    break;
  }
  params.line = std::move(line);
  if (params.line == "OK") {
    return EventT::OK;
  }
  if (params.line.rfind("OK ", 0) == 0) {
    params.remain = std::string{params.line}.substr(3);
    return EventT::OK;
  }
  if (params.line.rfind("EVER ", 0) == 0) {
    params.remain = std::string{params.line}.substr(5);
    return EventT::VER;
  }
  if (params.line.rfind("EVENT ", 0) == 0) {
    params.remain = std::string{params.line}.substr(6);
    auto beg = std::cbegin(params.remain);
    if (!arg::get_num8(beg, std::cend(params.remain), params.event.num)) {
      params.event.num = 0;
    }
    return EventT::EVENT;
  }
  if (params.line.rfind("ERXUDP ", 0) == 0) {
    params.remain = std::string{params.line}.substr(7);
    return EventT::RXUDP;
  }
  if (params.line.rfind("EPANDESC ", 0) == 0) {
    params.remain = std::string{params.line}.substr(9);
    return EventT::PANDESC;
  }
  return EventT::UNKNOWN;
}

}  // namespace b_route::libbp35

}  // namespace esphome
