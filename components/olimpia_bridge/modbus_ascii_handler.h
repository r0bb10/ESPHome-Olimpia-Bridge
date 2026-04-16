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

// --- Constants ---
static constexpr size_t MAX_QUEUE_SIZE = 30;
static constexpr size_t MAX_FRAME_BUFFER_SIZE = 128;
static constexpr uint8_t DEFAULT_RETRIES = 2;
static constexpr uint32_t FSM_TIMEOUT_MS = 500;
static constexpr uint32_t FSM_WATCHDOG_TIMEOUT_MS = 10000;
static constexpr uint32_t ACTIVITY_LED_ON_MS = 75;

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
  uint8_t retries_left;
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

  void set_en_pin(GPIOPin *pin) {
    this->en_pin_ = pin;
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

  void set_activity_pin(GPIOPin *pin) {
    this->activity_pin_ = pin;
  }

  bool is_ready() const {
    // Only one mode can be active
    bool has_en = (this->en_pin_ != nullptr);
    bool has_re = (this->re_pin_ != nullptr);
    bool has_de = (this->de_pin_ != nullptr);
    if (has_en && (has_re || has_de)) return false;
    if (has_en) return this->uart_ != nullptr && this->en_pin_ != nullptr;
    if (has_re && has_de) return this->uart_ != nullptr && this->re_pin_ != nullptr && this->de_pin_ != nullptr;
    return false;
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
  void drain_uart_rx_buffer_();

  // ASCII Frame Encoding/Decoding
  std::string encode_ascii_frame(const std::vector<uint8_t> &data);
  bool decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data);
  uint8_t compute_lrc(const std::vector<uint8_t> &data);

  // FSM Frame Building & Response
  bool read_available_();

  // FSM State
  ModbusState fsm_state_{ModbusState::IDLE};
  uint32_t fsm_start_time_{0};
  std::queue<ModbusRequest> request_queue_;
  ModbusRequest current_request_;
  std::vector<uint8_t> rx_buffer_;

  // Hardware Interfaces
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *en_pin_{nullptr};
  GPIOPin *re_pin_{nullptr};
  GPIOPin *de_pin_{nullptr};
  GPIOPin *activity_pin_{nullptr};
  uint32_t activity_led_off_time_{0};

  // Error tracking
  uint32_t total_requests_{0};
  uint32_t failed_requests_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
