#pragma once

#include <queue>
#include <vector>
#include <functional>
#include "esphome/core/gpio.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace olimpia_bridge {

class OlimpiaBridge;  // Forward declaration

// --- FSM States ---
enum class ModbusState {
  IDLE,
  SEND_REQUEST,
  WAIT_RESPONSE,
  PROCESS_RESPONSE,
  ERROR
};

// --- Modbus Request Structure ---
struct ModbusRequest {
  uint8_t address;
  uint8_t function;
  uint8_t retries_left = 2;
  uint16_t start_register;
  uint16_t length_or_value;
  bool is_write;
  std::function<void(bool success, std::vector<uint16_t> response)> callback;
};

// --- Modbus ASCII Handler Class ---
class ModbusAsciiHandler : public esphome::Component {
 public:
  ModbusAsciiHandler() = default;

  // Hardware Configuration
  void set_uart(uart::UARTComponent *uart) { 
    this->uart_ = uart;
    this->check_config_();
  }
  
  void set_re_pin(GPIOPin *pin) { 
    this->re_pin_ = pin;
    this->check_config_();
  }
  
  void set_de_pin(GPIOPin *pin) { 
    this->de_pin_ = pin;
    this->check_config_();
  }

  bool is_ready() const { 
    return this->uart_ != nullptr && this->re_pin_ != nullptr && this->de_pin_ != nullptr; 
  }

  void set_direction(bool transmit);  // true = TX, false = RX

  void setup() override;

  // Public Modbus API
  void add_request(ModbusRequest request);
  void read_register(uint8_t address, uint16_t reg, uint16_t count,
                     std::function<void(bool success, std::vector<uint16_t> response)> callback);

  void write_register(uint8_t address, uint16_t reg, uint16_t value,
                      std::function<void(bool success, std::vector<uint16_t> response)> callback);

  void loop() override;

  // Error handling
  float get_error_ratio() const;

 protected:
  // Configuration check
  void check_config_();

  // ASCII Frame Encoding/Decoding
  std::string encode_ascii_frame(const std::vector<uint8_t> &data);
  bool decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data);
  uint8_t compute_lrc(const std::vector<uint8_t> &data);

  // FSM Frame Building & Response
  bool read_available_();

  // FSM State
  ModbusState fsm_state_{ModbusState::IDLE};
  uint32_t fsm_start_time_{0};
  static constexpr uint32_t fsm_timeout_ms_{500};
  std::queue<ModbusRequest> request_queue_;
  ModbusRequest current_request_;
  std::vector<uint8_t> rx_buffer_;

  // Hardware Interfaces
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *re_pin_{nullptr};
  GPIOPin *de_pin_{nullptr};

  // Error tracking
  uint32_t total_requests_{0};
  uint32_t failed_requests_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
