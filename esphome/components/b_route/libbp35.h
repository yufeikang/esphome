#pragma once
#include <string>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace b_route {
namespace libbp35 {

enum class EventT {
  NONE,
  ERROR,
  VER,
  RXUDP,
  PANDESC,
  EVENT,
  OK,
  UNKNOWN,
};

extern const char *event_str(EventT ev);
extern const char *event_num_str(uint8_t num);

struct EventParamsT {
  std::string line;
  std::string remain;
  union {
    struct {
      uint8_t num;
    } event;
  };
  void clear() {
    line.clear();
    remain = {};
    event.num = 0;
  }
};

struct RxudpT {
  uint8_t sender[16];
  uint8_t dest[16];
  uint16_t rport;
  uint16_t lport;
  uint8_t sender_lla[8];
  bool secured;
  uint16_t data_len;
  std::string::size_type data_pos;
};

class SerialIO {
 public:
  virtual size_t write(const char *str) = 0;
  virtual size_t write(char) = 0;
  virtual size_t write(const char *str, size_t len) = 0;
  virtual int read();
};

class BP35 {
 public:
  BP35(SerialIO &stream) : stream_(stream) {}

  template<typename... Args> void send_sk(const std::string &cmd, Args... args) {
    int a[] = {0, (stream_.write(cmd.data(), cmd.size()), 0),
               (stream_.write(' '), stream_.write(args.data(), sizeof...(args)), 0)...};
    static_cast<void>(a);
    stream_.write("\r\n");
  }

  template<typename... Args> void send_prod(const std::string &cmd, Args... args) {
    int a[] = {0, (stream_.write(cmd.data(), cmd.size()), 0),
               (stream_.write(' '), stream_.write(args.data(), sizeof...(args)), 0)...};
    static_cast<void>(a);
    stream_.write("\r");
  }

  template<typename... Args>
  void send_sk_with_data(const std::string &cmd, const uint8_t *data, size_t data_len, const Args &...args) {
    int a[] = {0, (stream_.write(cmd.data(), cmd.size()), 0),
               (stream_.write(' '), stream_.write(args.data(), sizeof...(args)), 0)...};
    static_cast<void>(a);
    stream_.write(' ');
    stream_.write(reinterpret_cast<const char *>(data), data_len);
  }
  bool read_line(std::string &line, uint32_t timeout);
  EventT get_event(uint32_t timeout, EventParamsT &params);

  static bool parse_rxudp(const std::string &remains, RxudpT &out);

 private:
  SerialIO &stream_;
};

}  // namespace libbp35
}  // namespace b_route
}  // namespace esphome
