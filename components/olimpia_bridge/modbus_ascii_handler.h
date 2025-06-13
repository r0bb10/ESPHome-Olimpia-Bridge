#pragma once

#include <vector>
#include <string>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace olimpia_bridge {

class OlimpiaBridge;  // Forward declaration

class ModbusAsciiHandler {
 public:
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }

  // Set single directional pin instead of using 2
  void set_direction_pin(GPIOPin *pin) { this->direction_pin_ = pin; }

  // Set bridge for Modbus error tracking
  void set_bridge(OlimpiaBridge *bridge) { this->bridge_ = bridge; }

  // Set Modbus response timeout
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }

  bool send_and_receive(const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  bool read_register(uint8_t address, uint16_t reg, uint16_t *out);
  bool write_register(uint8_t address, uint16_t reg, uint16_t value);

  // Set separate RE/DE pins
  void set_re_pin(GPIOPin *pin) { this->re_pin_ = pin; }
  void set_de_pin(GPIOPin *pin) { this->de_pin_ = pin; }

 protected:
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *direction_pin_{nullptr};
  OlimpiaBridge *bridge_{nullptr};  // Used to report success/failure
  uint32_t timeout_ms_{200};        // Modbus receive timeout in ms

  GPIOPin *re_pin_{nullptr};   // Optional: separate receiver enable pin
  GPIOPin *de_pin_{nullptr};   // Optional: separate driver enable pin

  uint8_t lrc(const std::vector<uint8_t> &data);
  std::string encode_ascii_frame(const std::vector<uint8_t> &data);
  bool decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data);
};

}  // namespace olimpia_bridge
}  // namespace esphome