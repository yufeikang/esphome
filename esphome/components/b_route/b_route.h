#pragma once
#include <esphome/components/sensor/sensor.h>
#include <esphome/components/uart/uart.h>
#include <esphome/core/component.h>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include <cmath>
#include "bp35cmd.h"
#include "echonet_lite.h"
#include "libbp35.h"

namespace esphome {
namespace b_route {

using namespace echonet_lite;
using namespace libbp35;

class BRoute : public Component, public uart::UARTDevice, public SerialIO {
 public:
  BRoute();
  void loop() override;
  void set_power_sensor(sensor::Sensor *sensor) { power_sensor_ = sensor; }
  void set_energy_sensor(sensor::Sensor *sensor) { energy_sensor_ = sensor; }
  void set_power_sensor_interval_sec(uint32_t interval) { power_sensor_interval_ = interval * 1000; }
  void set_energy_sensor_interval_sec(uint32_t interval) { energy_sensor_interval_ = interval * 1000; }
  void set_rejoin_miss_count(uint8_t count) { rejoin_miss_count_ = count; }
  void set_rejoin_timeout_sec(uint32_t sec) { rejoin_timeout_ = sec * 1000; }
  void set_rescan_timeout_sec(uint32_t sec) { rescan_timeout_ = sec * 1000; }
  void set_restart_timeout_sec(uint32_t sec) { reboot_timeout_ = sec * 1000; }
  void set_rbid(const char *id, const char *password) {
    rb_id_ = id;
    rb_password_ = password;
  }

  size_t write(char c) override {
    write_byte(c);
    return 1;
  }
  size_t write(const char *str) override {
    size_t len = std::strlen(str);
    write_array(reinterpret_cast<const uint8_t *>(str), len);
    return len;
  }
  size_t write(const char *data, size_t len) override {
    write_array(reinterpret_cast<const uint8_t *>(data), len);
    return len;
  }

  int read() override {
    if (available() < 1) {
      return -1;
    }
    uint8_t b;
    if (read_byte(&b)) {
      return b;
    } else {
      return -1;
    }
  }

 private:
  static constexpr EOJ EOJ_CONTROLLER{0x05, 0xff, 0x01};
  static constexpr EOJ EOJ_LOWV_SMART_METER{0x02, 0x88, 0x01};
  static constexpr uint32_t REQUEST_PROPERTY_INTERVAL = 5'000;
  static constexpr const char *TAG = "b_route";

  enum class InitialValueT { PWD, RBID, PANID, CHANNEL, ROPT, WOPT, ECHO } setting_value_ = InitialValueT::PWD;
  enum class StateT {
    INIT,
    WAIT_VER,
    SETTING_VALUES,
    SCANNING,
    JOINING,
    RUNNING,
    ADDR_CONV,
    RESTARTING
  } state_ = StateT::INIT;

  libbp35::BP35 bp_{*this};
  sensor::Sensor *power_sensor_ = nullptr;
  sensor::Sensor *energy_sensor_ = nullptr;
  std::string v6_address_;
  std::string channel_;
  std::string panid_;
  std::string mac_;
  const char *rb_password_ = nullptr;
  const char *rb_id_ = nullptr;

  int32_t energy_coeff_ = -1;
  float energy_unit_ = NAN;
  uint32_t property_requested_ = 0;
  uint32_t state_timeout_ = 0;
  uint32_t state_started_ = 0;
  uint32_t rejoin_timer_ = 0;
  uint32_t rescan_timer_ = 0;
  uint32_t reboot_timer_ = 0;
  uint8_t miss_count_ = 0;
  uint32_t power_sensor_interval_ = 30'000;
  uint32_t energy_sensor_interval_ = 60'000;
  uint32_t rejoin_timeout_ = 0;
  uint32_t rescan_timeout_ = 0;
  uint32_t reboot_timeout_ = 0;
  uint8_t rejoin_miss_count_ = 0;

  void set_state_(StateT state, uint32_t timeout);
  void start_join_();
  void start_scan_();
  void handle_rxudp_(std::string &hexstr);
  void handle_property_response_(const uint8_t *raw, const echonet_lite::Packet &pkt);
  void request_momentary_power_();
  void request_integral_energy_();
  void request_energy_parameters_();
  bool test_nw_info_() const;
  bool energy_params_received_() const { return std::isfinite(energy_unit_) && energy_coeff_ > 0; }
  EventT get_event_(EventParamsT &params);
  void setup() override;
  std::array<uint8_t, 255> out_buffer_{};
  bool is_measurement_requesting_() const {
    return (power_sensor_ && power_sensor_interval_ > 0 && power_sensor_interval_ != esphome::SCHEDULER_DONT_RUN) ||
           (energy_sensor_ && energy_sensor_interval_ > 0 && energy_sensor_interval_ != esphome::SCHEDULER_DONT_RUN);
  }
  void reset_timers_() { rejoin_timer_ = rescan_timer_ = reboot_timer_ = esphome::millis(); }
  template<size_t N> bool request_property_(std::array<uint8_t, N> props);
  static const char *state_name(StateT /*state*/);
};

}  // namespace b_route
}  // namespace esphome
