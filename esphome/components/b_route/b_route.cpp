#include "b_route.h"

#include <cmath>
#include <cstring>
#include <esphome/core/application.h>
#include <utility>
#include <cstdint>

namespace esphome {
namespace b_route {

using namespace cmd;
using namespace libbp35;

namespace echo = echonet_lite;
namespace meter = echonet_lite::props::lowv_smart_meter;

namespace {

std::array<uint8_t, 255> const BUFFER{};
constexpr std::array PROPS_MOMENTARY_POWER{meter::MOMENTARY_POWER};
constexpr std::array PROPS_ENERGY_PARAMS{meter::ENERGY_COEFF, meter::ENERGY_UNIT};
constexpr std::array PROPS_INTEGRAL_ENERGY{meter::INTEGRAL_ENERGY_FWD};

constexpr uint32_t SEND_RETRY_INTERVAL = 2'000;
constexpr uint32_t REQUEST_RETRY_INTERVAL = 5'000;
constexpr uint32_t RESTART_DELAY = 5'000;

constexpr const char *POWER_TASK = "power";
constexpr const char *ENERGY_TASK = "energy";
constexpr const char *PARAMS_TASK = "params";

constexpr const char *SCAN_KEY_ADDR = "Addr:";
constexpr const char *SCAN_KEY_PANID = "Pan ID:";
constexpr const char *SCAN_KEY_CHANNEL = "Channel:";

}  // namespace

BRoute::BRoute() {}

void BRoute::request_energy_parameters_() {
  auto rc = request_property_(PROPS_ENERGY_PARAMS);
  if (rc) {
    ESP_LOGD(TAG, "Energy params requested");
  }
  App.scheduler.set_timeout(this, PARAMS_TASK, rc ? REQUEST_RETRY_INTERVAL : SEND_RETRY_INTERVAL,
                            [this] { request_energy_parameters_(); });
}

void BRoute::request_momentary_power_() {
  auto rc = request_property_(PROPS_MOMENTARY_POWER);
  if (rc) {
    ESP_LOGD(TAG, "POWER requested");
  }
  App.scheduler.set_timeout(this, POWER_TASK, rc ? REQUEST_RETRY_INTERVAL : SEND_RETRY_INTERVAL,
                            [this] { request_momentary_power_(); });
}

void BRoute::request_integral_energy_() {
  auto rc = energy_params_received_() && request_property_(PROPS_INTEGRAL_ENERGY);
  if (rc) {
    ESP_LOGD(TAG, "ENERGY requested");
  }
  App.scheduler.set_timeout(this, ENERGY_TASK, rc ? REQUEST_RETRY_INTERVAL : SEND_RETRY_INTERVAL,
                            [this] { request_integral_energy_(); });
}

bool BRoute::test_nw_info_() const {
  ESP_LOGD(TAG, "scan data: mac=%s, panid=%s, channel=%s", mac_.c_str(), panid_.c_str(), channel_.c_str());
  if (mac_.empty() && panid_.empty() && channel_.empty()) {
    return false;
  }
  if (mac_.length() != 16 || panid_.length() != 4 || channel_.length() != 2) {
    ESP_LOGE(TAG, "Unexpected scan data: mac=%s, panid=%s, channel=%s", mac_.c_str(), panid_.c_str(), channel_.c_str());
    return false;
  }
  return true;
}

void BRoute::setup() {
  if (parent_ == nullptr) {
    ESP_LOGE(TAG, "No serial specified");
    mark_failed();
    return;
  }
  if (rb_id_ == nullptr || rb_password_ == nullptr) {
    ESP_LOGE(TAG, "Route B ID/Password not set");
    mark_failed();
    return;
  }
  if (power_sensor_ || energy_sensor_) {
    request_energy_parameters_();
  }
  if (power_sensor_ && power_sensor_interval_) {
    set_interval(power_sensor_interval_, [this] { request_momentary_power_(); });
  }
  if (energy_sensor_ && energy_sensor_interval_) {
    set_interval(energy_sensor_interval_, [this] { request_integral_energy_(); });
  }
  reset_timers_();
}

void BRoute::handle_property_response_(const uint8_t *raw, const echo::Packet &pkt) {
  for (int i = 0; i < pkt.opc; i++) {
    auto &prop = pkt.properties[i];
    if (prop.epc == meter::ENERGY_COEFF) {
      ESP_LOGD(TAG, "coeff received");
      if (pkt.esv == static_cast<uint8_t>(echo::ESV::GET_SNA) && prop.pdc == 0) {
        energy_coeff_ = 1;
        miss_count_ = 0;
      } else {
        if (prop.pdc != sizeof(energy_coeff_)) {
          ESP_LOGW(TAG, "property(coeff) len mismatch %u != %u", prop.pdc, sizeof(energy_coeff_));
          continue;
        }
        miss_count_ = 0;
        energy_coeff_ = echo::Codec::get_signed_long(raw + prop.offset);
      }
      if (energy_params_received_()) {
        App.scheduler.cancel_timeout(this, PARAMS_TASK);
      }
      continue;
    } else if (prop.epc == meter::ENERGY_UNIT) {
      ESP_LOGD(TAG, "unit received");
      if (prop.pdc != 1) {
        ESP_LOGW(TAG, "Property(unit) len mismatch %u != 1", prop.pdc);
        continue;
      }
      miss_count_ = 0;
      auto v = static_cast<int8_t>(raw[prop.offset]);
      energy_unit_ = v > 10 ? std::pow(10.0f, v - 9) : std::pow(10.0f, -v);
      if (energy_params_received_()) {
        App.scheduler.cancel_timeout(this, PARAMS_TASK);
        if (energy_sensor_) {
          int8_t prec = 0;
          if (v < 10) {
            prec = static_cast<int>(std::ceil(v - std::log10(static_cast<float>(energy_coeff_))));
            energy_sensor_->set_accuracy_decimals(prec);
          }
        }
      }
      continue;
    }
    if (pkt.esv == static_cast<uint8_t>(echo::ESV::GET_SNA)) {
      continue;
    }
    if (prop.epc == meter::MOMENTARY_POWER) {
      ESP_LOGD(TAG, "POWER received");
      App.scheduler.cancel_timeout(this, POWER_TASK);
      int32_t power;
      if (prop.pdc != sizeof(power)) {
        ESP_LOGW(TAG, "Property(momentary power) len mismatch %u != %u", prop.pdc, sizeof(power));
        continue;
      }
      reset_timers_();
      miss_count_ = 0;
      if (power_sensor_) {
        power = echo::Codec::get_signed_long(raw + prop.offset);
        power_sensor_->publish_state(power);
      }
    } else if (prop.epc == meter::SCHEDULED_INTEGRAL_ENERGY_FWD) {
      ESP_LOGD(TAG, "Scheduled ENERGY received");
      echo::IntegralPowerWithDateTime data;
      if (prop.pdc != sizeof(data)) {
        ESP_LOGW(TAG, "Property(sched integral energy) len mismatch %u != %u", prop.pdc, sizeof(data));
        continue;
      }
      reset_timers_();
      data.year = echo::Codec::get_unsigned_short(raw + prop.offset);
      std::copy(raw + prop.offset + offsetof(echo::IntegralPowerWithDateTime, mon),
                raw + prop.offset + offsetof(echo::IntegralPowerWithDateTime, value),
                reinterpret_cast<uint8_t *>(&data) + offsetof(echo::IntegralPowerWithDateTime, mon));
      data.value = echo::Codec::get_unsigned_long(raw + prop.offset + offsetof(echo::IntegralPowerWithDateTime, value));
      ESP_LOGI(TAG, "Integral data of %02u:%02u received", data.hour, data.min);
    } else if (prop.epc == meter::INTEGRAL_ENERGY_FWD) {
      ESP_LOGD(TAG, "ENERGY received");
      App.scheduler.cancel_timeout(this, ENERGY_TASK);
      uint32_t evalue;
      if (prop.pdc != sizeof(evalue)) {
        ESP_LOGW(TAG, "Property(integral energy fwd) len mismatch %u != %u", prop.pdc, sizeof(evalue));
        continue;
      }
      reset_timers_();
      miss_count_ = 0;
      if (energy_sensor_) {
        evalue = echo::Codec::get_unsigned_long(raw + prop.offset);
        auto fenergy = energy_unit_ * evalue * energy_coeff_;
        ESP_LOGV(TAG, "Energy %.3f = %.4f(kWh) * %u * %d, prec=%d", fenergy, energy_unit_, evalue, energy_coeff_,
                 energy_sensor_->get_accuracy_decimals());
        energy_sensor_->publish_state(fenergy);
      }
    } else {
      ESP_LOGD(TAG, "Drop property response %02X", prop.epc);
    }
  }
}

const char *BRoute::state_name(StateT state) {
  switch (state) {
    case StateT::INIT:
      return "init";
    case StateT::JOINING:
      return "joining";
    case StateT::RUNNING:
      return "running";
    case StateT::SCANNING:
      return "scanning";
    case StateT::SETTING_VALUES:
      return "settings";
    case StateT::WAIT_VER:
      return "ver";
    case StateT::ADDR_CONV:
      return "addr_conv";
    case StateT::RESTARTING:
      return "restarting";
    default:
      return "unknown";
  }
}

void BRoute::set_state_(StateT state, uint32_t timeout) {
  if (this->state_ == StateT::RESTARTING) {
    return;
  }
  this->state_ = state;
  state_timeout_ = timeout;
  state_started_ = esphome::millis();
}

void BRoute::start_join_() {
  bp_.send_sk("SKJOIN", arg::str(v6_address_));
  set_state_(StateT::JOINING, 10'000);
}

void BRoute::start_scan_() {
  mac_.clear();
  panid_.clear();
  channel_.clear();
  bp_.send_sk("SKSCAN 2 FFFFFFFF 6");
  set_state_(StateT::SCANNING, 20'000);
}

void BRoute::handle_rxudp_(std::string &hexstr) {
  ESP_LOGV(TAG, "RXUDP: %s", hexstr.data());
  auto start = esphome::millis();
  RxudpT rxudp;
  if (!BP35::parse_rxudp(std::move(hexstr), rxudp)) {
    ESP_LOGW(TAG, "%s: Failed to parse rxudp, skipped", hexstr.data());
    return;
  }
  if (rxudp.lport != echo::UDP_PORT) {
    ESP_LOGD(TAG, "%u: Destination port is not for EchonetLite", rxudp.lport);
    return;
  }
  ESP_LOGV(TAG, "udp data len = %u, datastr = %s", rxudp.data_len, hexstr.data() + rxudp.data_pos);
  size_t len;
  if (!util::hex2bin(&hexstr[rxudp.data_pos], BUFFER, len) || len != rxudp.data_len) {
    ESP_LOGW(TAG, "%s: Failed to decode udp data", &hexstr[rxudp.data_pos]);
    return;
  }
  echo::Packet pkt;
  if (!echo::Codec::decode_packet(BUFFER.data(), len, pkt)) {
    ESP_LOGW(TAG, "%s: Failed to decode echonet packet", &hexstr[rxudp.data_pos]);
    return;
  }
  if (pkt.ehd1 != echo::EHD1 || pkt.ehd2 != echo::EH_D2_FORMAT1) {
    return;
  }
  // handle low power smart meter
  if (pkt.seoj.X1 != 0x02 || pkt.seoj.X2 != 0x88) {
    return;
  }
  ESP_LOGV(TAG, "Echonet ehd=%02x,%02x deoj=%02x%02x%02x, esv=%02x, npc=%u, epc[0]=%02x", pkt.ehd1, pkt.ehd2,
           pkt.deoj.X1, pkt.deoj.X2, pkt.deoj.X3, pkt.esv, pkt.opc, pkt.opc == 0 ? -1 : pkt.properties[0].epc);
  if (pkt.esv == static_cast<uint8_t>(echo::ESV::GET_RES) || pkt.esv == static_cast<uint8_t>(echo::ESV::INF) ||
      pkt.esv == static_cast<uint8_t>(echo::ESV::GET_SNA)) {
    handle_property_response_(BUFFER.data(), pkt);
  }
}

libbp35::EventT BRoute::get_event_(EventParamsT &params) {
  auto ev = bp_.get_event(100, params);
  if (ev != EventT::NONE) {
    if (ev == EventT::EVENT) {
      ESP_LOGV(TAG, "ev = %s(%s)", libbp35::event_str(ev), libbp35::event_num_str(params.event.num));
    } else {
      ESP_LOGV(TAG, "ev = %s, line=%s", libbp35::event_str(ev), params.line.c_str());
    }
  }
  return ev;
}

void BRoute::loop() {
  if (reboot_timeout_ && is_measurement_requesting_()) {
    if (auto elapsed = esphome::millis() - reboot_timer_; elapsed > reboot_timeout_) {
      ESP_LOGE(TAG, u8"計測データを %u 秒間受信していません。再起動します", elapsed / 1000);
      set_state_(StateT::RESTARTING, 0);
      return;
    }
  }
  EventParamsT params{};
  auto ev = get_event_(params);
  switch (state_) {
    case StateT::RESTARTING:
      if (esphome::millis() - state_started_ >= RESTART_DELAY) {
        mark_failed();
        App.safe_reboot();
      }
      break;
    case StateT::INIT:
      bp_.send_sk("SKVER");
      set_state_(StateT::WAIT_VER, 1'000);
      break;
    case StateT::WAIT_VER:
      if (ev == EventT::VER) {
        ESP_LOGD(TAG, "VER=%s", params.remain.data());
      } else if (ev == EventT::OK) {
        // disable echo back
        bp_.send_sk("SKSREG", arg::reg(0xfe), arg::mode(0));
        set_state_(StateT::SETTING_VALUES, 1'000);
        setting_value_ = InitialValueT::ECHO;
      }
      break;
    case StateT::SETTING_VALUES:
      if (ev == EventT::OK) {
        switch (setting_value_) {
          case InitialValueT::ECHO:
            // test binary mode
            bp_.send_prod("ROPT");
            setting_value_ = InitialValueT::ROPT;
            break;
          case InitialValueT::ROPT:
            ESP_LOGD(TAG, "ropt=%s", params.remain.data());
            if (params.remain != "01") {
              // set to ascii mode
              bp_.send_prod("WOPT", arg::num8(1));
              setting_value_ = InitialValueT::WOPT;
              break;
            } else {
              [[fallthrough]];
            }
          case InitialValueT::WOPT:
            bp_.send_sk("SKSETPWD", arg::num8(std::strlen(rb_password_)), arg::str(rb_password_));
            set_state_(StateT::SETTING_VALUES, 1'000);
            setting_value_ = InitialValueT::PWD;
            break;
          case InitialValueT::PWD:
            bp_.send_sk("SKSETRBID", arg::str(rb_id_));
            setting_value_ = InitialValueT::RBID;
            break;
          case InitialValueT::RBID:
            start_scan_();
            break;
          case InitialValueT::CHANNEL:
            bp_.send_sk("SKSREG", arg::reg(0x03), arg::str(panid_));
            setting_value_ = InitialValueT::PANID;
            break;
          case InitialValueT::PANID:
            start_join_();
            break;
          default:
            ESP_LOGE(TAG, "%d: Unexpected setting value type", static_cast<int>(setting_value_));
            break;
        }
      }
      break;
    case StateT::ADDR_CONV:
      if (ev == EventT::UNKNOWN) {
        if (params.line.rfind("SKLL", 0) == 0) {
          break;
        }
        if (params.line.length() == 39) {
          v6_address_ = params.line;
          bp_.send_sk("SKSREG", arg::reg(0x02), arg::str(channel_));
          setting_value_ = InitialValueT::CHANNEL;
          set_state_(StateT::SETTING_VALUES, 1'000);
        }
      }
      break;
    case StateT::SCANNING:
      if (ev == EventT::OK) {
        ESP_LOGI(TAG, "Scanning...");
      } else if (ev == EventT::EVENT && params.event.num == 0x22) {
        if (test_nw_info_()) {
          ESP_LOGI(TAG, "Scan done");
          rescan_timer_ = esphome::millis();
          bp_.send_sk("SKLL64", arg::str(mac_));
          set_state_(StateT::ADDR_CONV, 1'000);
        } else {
          ESP_LOGW(TAG, "Scan done but channel not received, scan again");
          start_scan_();
        }
        break;
      } else if (ev == EventT::UNKNOWN && params.line[0] == ' ') {
        auto line = util::trim_sv(params.line);
        if (line.rfind(SCAN_KEY_ADDR, 0) == 0) {
          mac_ = line.substr(SCAN_KEY_ADDR.length());
        } else if (line.rfind(SCAN_KEY_PANID, 0) == 0) {
          panid_ = line.substr(SCAN_KEY_PANID.length());
        } else if (line.rfind(SCAN_KEY_CHANNEL, 0) == 0) {
          channel_ = line.substr(SCAN_KEY_CHANNEL.length());
        }
      }
      break;
    case StateT::JOINING:
      if (ev == EventT::OK) {
        ESP_LOGI(TAG, "Joining...");
      } else if (ev == EventT::EVENT) {
        if (params.event.num == 0x25) {
          ESP_LOGI(TAG, "Joined");
          set_state_(StateT::RUNNING, 0);
          rejoin_timer_ = esphome::millis();
        } else if (params.event.num == 0x24) {
          ESP_LOGW(TAG, "Failed to join, try scan and join");
          start_scan_();
        } else {
          if (params.event.num != 0x21) {
            ESP_LOGD(TAG, "%02x: Ignore event", params.event.num);
          }
        }
      }
      break;
    case StateT::RUNNING: {
      switch (ev) {
        case EventT::EVENT:
          switch (params.event.num) {
            case 0x32:
              ESP_LOGI(TAG, "Transmit time limit activated");
              break;
            case 0x33:
              ESP_LOGW(TAG, "Transmit time limit cleared");
              break;
            case 0x29:
              ESP_LOGI(TAG, "Session expired, waiting re-join");
              set_state_(StateT::JOINING, 10'000);
              break;
            default:
              ESP_LOGV(TAG, "%02x: Unhandled event", params.event.num);
              break;
          }
          break;
        case EventT::RXUDP:
          handle_rxudp_(params.remain);
          break;
        case EventT::NONE:
          break;
        default:
          ESP_LOGV(TAG, "%d: Unhandled input", static_cast<int>(ev));
          break;
      }
      if (rescan_timeout_ && is_measurement_requesting_()) {
        if (auto elapsed = esphome::millis() - rescan_timer_; elapsed > rescan_timeout_) {
          ESP_LOGE(TAG, u8"計測データを %u 秒間受信していません。再スキャンします", elapsed / 1000);
          start_scan_();
          break;
        }
      }
      if (rejoin_timeout_ && is_measurement_requesting_()) {
        if (auto elapsed = esphome::millis() - rejoin_timer_; elapsed > rejoin_timeout_) {
          ESP_LOGI(TAG, u8"計測データを %u 秒間受信していません。再接続します", elapsed / 1000);
          start_join_();
          break;
        }
      }
    } break;
    default:
      ESP_LOGD(TAG, "%d: Unhandled state", static_cast<int>(state_));
      break;
  }
  if (state_timeout_ && esphome::millis() - state_started_ > state_timeout_) {
    ESP_LOGW(TAG, "%s: State timeout, re-run from init", state_name(state_));
    set_state_(StateT::INIT, 0);
  }
}
}  // namespace b_route
}  // namespace esphome
