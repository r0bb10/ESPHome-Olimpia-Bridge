#include "esphome/core/log.h"
#include "modbus_ascii_handler.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "modbus_ascii";

void ModbusAsciiHandler::check_config_() {
  if (this->is_ready()) {
    ESP_LOGCONFIG(TAG, "[Modbus] ModbusAsciiHandler configuration complete");
  }
}

// --- Component Setup ---
void ModbusAsciiHandler::setup() {
  if (this->uart_ == nullptr) {
    ESP_LOGE(TAG, "[Modbus] No UART configured for ModbusAsciiHandler");
    this->mark_failed();
    return;
  }

  // Check for conflicting or missing direction pin configuration
  bool has_en = (this->en_pin_ != nullptr);
  bool has_re = (this->re_pin_ != nullptr);
  bool has_de = (this->de_pin_ != nullptr);

  if (has_en && (has_re || has_de)) {
    ESP_LOGE(TAG, "[Modbus] Invalid configuration: en_pin cannot be used with re_pin or de_pin");
    this->mark_failed();
    return;
  }

  if (has_en) {
    this->en_pin_->setup();
    this->en_pin_->digital_write(false); // RX mode by default (driver disabled)
    ESP_LOGCONFIG(TAG, "[Modbus] RS-485 single EN direction pin initialized");
  } else if (has_re && has_de) {
    this->re_pin_->setup();
    this->de_pin_->setup();
    this->re_pin_->digital_write(false);  // RX mode (disable driver)
    this->de_pin_->digital_write(false);  // RX mode (disable driver)
    ESP_LOGCONFIG(TAG, "[Modbus] RS-485 RE/DE direction control pins initialized");
  } else {
    ESP_LOGE(TAG, "[Modbus] No valid direction control pin(s) configured for RS-485");
    this->mark_failed();
    return;
  }

  // Activity LED setup (inverted logic: HIGH = off, LOW = on)
  if (this->activity_pin_ != nullptr) {
    this->activity_pin_->setup();
    this->activity_pin_->digital_write(true); // LED off by default
  }
}

// --- Direction control ---
void ModbusAsciiHandler::set_direction(bool transmit) {
  if (this->en_pin_ != nullptr) {
    this->en_pin_->digital_write(transmit);
  } else {
    if (this->re_pin_ != nullptr) {
      this->re_pin_->digital_write(transmit);
    }
    if (this->de_pin_ != nullptr) {
      this->de_pin_->digital_write(transmit);
    }
  }
}

void ModbusAsciiHandler::drain_uart_rx_buffer_() {
  if (this->uart_ == nullptr)
    return;

  while (this->uart_->available()) {
    uint8_t byte;
    if (!this->uart_->read_byte(&byte))
      break;
  }
}

// --- LRC checksum ---
uint8_t ModbusAsciiHandler::compute_lrc(const std::vector<uint8_t> &data) {
  uint8_t sum = 0;
  for (uint8_t b : data)
    sum += b;
  return static_cast<uint8_t>(-sum);
}

// --- Error Ratio ---
float ModbusAsciiHandler::get_error_ratio() const {
  if (this->total_requests_ == 0) return 0.0f;
  return static_cast<float>(this->failed_requests_ * 100) / this->total_requests_;
}

// --- Hex helper ---
static constexpr char HEX_CHARS[] = "0123456789ABCDEF";

inline uint8_t hex_char_to_val(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

// --- ASCII encoding ---
std::string ModbusAsciiHandler::encode_ascii_frame(const std::vector<uint8_t> &data) {
  std::string result;
  result.reserve(1 + 2 * (data.size() + 1) + 2);
  result.push_back(':');

  for (uint8_t byte : data) {
    result.push_back(HEX_CHARS[(byte >> 4) & 0x0F]);
    result.push_back(HEX_CHARS[byte & 0x0F]);
  }

  uint8_t lrc = compute_lrc(data);
  result.push_back(HEX_CHARS[(lrc >> 4) & 0x0F]);
  result.push_back(HEX_CHARS[lrc & 0x0F]);
  result += "\r\n";

  std::string printable = result;
  if (printable.size() >= 2 && printable.substr(printable.size() - 2) == "\r\n")
    printable.erase(printable.size() - 2);

  ESP_LOGD(TAG, "[Modbus] Encoded ASCII frame: %s\\r\\n", printable.c_str());
  return result;
}

// --- ASCII decoding ---
bool ModbusAsciiHandler::decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data) {
  if (frame.size() < 7 || frame[0] != ':' || frame.substr(frame.size() - 2) != "\r\n") {
    ESP_LOGW(TAG, "[Modbus] Invalid frame start/end");
    return false;
  }

  size_t payload_len = frame.size() - 3;
  if (payload_len % 2 != 0) {
    ESP_LOGW(TAG, "[Modbus] Invalid frame length");
    return false;
  }

  data.clear();
  data.reserve(payload_len / 2 - 1);

  uint8_t sum = 0;
  for (size_t i = 1; i < frame.size() - 4; i += 2) {
    uint8_t hi = hex_char_to_val(frame[i]);
    uint8_t lo = hex_char_to_val(frame[i + 1]);
    if (hi == 0xFF || lo == 0xFF) {
      ESP_LOGW(TAG, "[Modbus] Invalid hex chars '%c%c' at pos %zu", frame[i], frame[i + 1], i);
      return false;
    }
    uint8_t byte = (hi << 4) | lo;
    data.push_back(byte);
    sum += byte;
  }

  uint8_t hi_lrc = hex_char_to_val(frame[frame.size() - 4]);
  uint8_t lo_lrc = hex_char_to_val(frame[frame.size() - 3]);
  if (hi_lrc == 0xFF || lo_lrc == 0xFF) {
    ESP_LOGW(TAG, "[Modbus] Invalid LRC hex chars '%c%c'", frame[frame.size() - 4], frame[frame.size() - 3]);
    return false;
  }

  uint8_t received_lrc = (hi_lrc << 4) | lo_lrc;
  uint8_t computed_lrc = static_cast<uint8_t>(-sum);

  if (computed_lrc != received_lrc) {
    ESP_LOGW(TAG, "[Modbus] LRC mismatch: computed=0x%02X received=0x%02X", computed_lrc, received_lrc);
    return false;
  }

  ESP_LOGD(TAG, "[Modbus] Decoded ASCII frame successfully");
  return true;
}

// --- Public API ---
void ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint16_t count,
  std::function<void(bool, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x03;
  req.start_register = reg;
  req.length_or_value = count;
  req.is_write = false;
  req.callback = std::move(callback);
  this->add_request(req);
}

void ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value,
  std::function<void(bool, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x06;
  req.start_register = reg;
  req.length_or_value = value;
  req.is_write = true;
  req.callback = std::move(callback);
  this->add_request(req);
}

// --- Add request to queue ---
void ModbusAsciiHandler::add_request(ModbusRequest request) {
  if (this->request_queue_.size() >= MAX_QUEUE_SIZE) {
    ESP_LOGW(TAG, "[FSM] Request queue full, rejecting new request (fn=0x%02X reg=0x%04X)",
             request.function, request.start_register);
    if (request.callback)
      request.callback(false, {});
    return;
  }

  request.retries_left = DEFAULT_RETRIES;
  this->request_queue_.push(std::move(request));
  ESP_LOGD(TAG, "[FSM] Request enqueued (fn=0x%02X reg=0x%04X)", request.function, request.start_register);
}

// --- FSM main loop ---
void ModbusAsciiHandler::loop() {
  static uint32_t last_state_change = 0;

  // Watchdog: detect stuck FSM (non-IDLE state for >10 seconds)
  if (this->fsm_state_ != ModbusState::IDLE) {
    uint32_t elapsed = millis() - last_state_change;
    if (elapsed > FSM_WATCHDOG_TIMEOUT_MS) {
      ESP_LOGW(TAG, "[FSM] Warning: FSM stuck in state %d for %u ms", this->fsm_state_, elapsed);
      this->fsm_state_ = ModbusState::ERROR;
      last_state_change = millis();  // Prevent repeated warnings
    }
  }

  // Activity LED: turn off if time expired (inverted logic)
  if (this->activity_pin_ != nullptr && this->activity_led_off_time_ > 0) {
    if (static_cast<int32_t>(millis() - this->activity_led_off_time_) >= 0) {
      this->activity_pin_->digital_write(true); // LED off
      this->activity_led_off_time_ = 0;
    }
  }

  // Nothing to do if idle and queue is empty
  if (this->fsm_state_ == ModbusState::IDLE && this->request_queue_.empty())
    return;

  switch (this->fsm_state_) {

    // --- FSM State: IDLE ---
    // Wait for a request; if one is available, dequeue and start sending
    case ModbusState::IDLE:
      if (!this->request_queue_.empty()) {
        this->current_request_ = this->request_queue_.front();
        this->request_queue_.pop();
        ESP_LOGD(TAG, "[FSM] Transition: IDLE → SEND_REQUEST");
        this->fsm_state_ = ModbusState::SEND_REQUEST;
        last_state_change = millis();
        this->total_requests_++;  // Increment total requests
      }
      break;

    // --- FSM State: SEND_REQUEST ---
    // Build and send the Modbus ASCII frame over UART
    case ModbusState::SEND_REQUEST: {
      // Drop any stale UART bytes from earlier activity before starting a new transaction.
      this->drain_uart_rx_buffer_();
      this->rx_buffer_.clear();

      std::vector<uint8_t> pdu = {
        this->current_request_.address,
        this->current_request_.function,
        static_cast<uint8_t>((this->current_request_.start_register >> 8) & 0xFF),
        static_cast<uint8_t>(this->current_request_.start_register & 0xFF),
        static_cast<uint8_t>((this->current_request_.length_or_value >> 8) & 0xFF),
        static_cast<uint8_t>(this->current_request_.length_or_value & 0xFF),
      };

      std::string frame_ascii = this->encode_ascii_frame(pdu);

      // Strip CRLF for logging readability
      std::string printable = frame_ascii;
      if (printable.size() >= 2 && printable.substr(printable.size() - 2) == "\r\n")
        printable.erase(printable.size() - 2);

      ESP_LOGD(TAG, "[FSM] TX ASCII Frame: %s\\r\\n", printable.c_str());

      this->set_direction(true);
      this->uart_->write_str(frame_ascii.c_str());
      this->uart_->flush();
      this->set_direction(false);

      // Start collecting a fresh response frame after the bus direction flips back to RX.
      this->rx_buffer_.clear();

      // Activity LED: turn on for activity
      if (this->activity_pin_ != nullptr) {
        this->activity_pin_->digital_write(false); // LED on (inverted)
        this->activity_led_off_time_ = millis() + ACTIVITY_LED_ON_MS;
      }

      this->fsm_start_time_ = millis();
      this->fsm_state_ = ModbusState::WAIT_RESPONSE;
      last_state_change = millis();
      break;
    }

    // --- FSM State: WAIT_RESPONSE ---
    // Wait for full frame from device; timeout after delay
    case ModbusState::WAIT_RESPONSE:
      if (this->read_available_()) {
        ESP_LOGD(TAG, "[FSM] Response ready, processing");
        this->fsm_state_ = ModbusState::PROCESS_RESPONSE;
        last_state_change = millis();
      } else if (millis() - this->fsm_start_time_ > FSM_TIMEOUT_MS) {
        ESP_LOGW(TAG, "[FSM] Timeout waiting for response");
        this->fsm_state_ = ModbusState::ERROR;
        last_state_change = millis();
      }
      break;

    // --- FSM State: PROCESS_RESPONSE ---
    // Parse the response frame, verify contents and call callback with results
    case ModbusState::PROCESS_RESPONSE: {
      std::string rx_string(this->rx_buffer_.begin(), this->rx_buffer_.end());
      this->rx_buffer_.clear();

      // Strip CRLF for logging readability and avoid terminal control chars
      std::string printable = rx_string;
      if (printable.size() >= 2 && printable.substr(printable.size() - 2) == "\r\n")
        printable.erase(printable.size() - 2);
      ESP_LOGD(TAG, "[FSM] RX ASCII Frame: %s\\r\\n", printable.c_str());

      std::vector<uint8_t> response;
      bool ok = this->decode_ascii_frame(rx_string, response);
      if (!ok) {
        ESP_LOGW(TAG, "[FSM] Invalid ASCII frame — decode failed");
        this->fsm_state_ = ModbusState::ERROR;
        last_state_change = millis();
        break;
      }

      std::vector<uint16_t> result;

      // Activity LED: turn on for activity
      if (this->activity_pin_ != nullptr) {
        this->activity_pin_->digital_write(false); // LED on (inverted)
        this->activity_led_off_time_ = millis() + ACTIVITY_LED_ON_MS;
      }

      // Handle 0x03 / 0x04 (read) responses
      if (this->current_request_.function == 0x03 || this->current_request_.function == 0x04) {
        if (response.size() < 3) {
          ESP_LOGW(TAG, "[FSM] Response too short");
          this->fsm_state_ = ModbusState::ERROR;
          last_state_change = millis();
          break;
        }

        uint8_t byte_count = response[2];
        if (response.size() < 3 + byte_count) {
          ESP_LOGW(TAG, "[FSM] Incomplete data payload");
          this->fsm_state_ = ModbusState::ERROR;
          last_state_change = millis();
          break;
        }

        for (size_t i = 0; i + 1 < byte_count; i += 2) {
          uint16_t val = (response[3 + i] << 8) | response[3 + i + 1];
          result.push_back(val);
        }
      }

      // Handle 0x06 (write) echo response
      else if (this->current_request_.function == 0x06) {
        if (response.size() != 6) {
          ESP_LOGW(TAG, "[FSM] Unexpected response length for 0x06 (expected 6, got %d)",
                   static_cast<int>(response.size()));
          this->fsm_state_ = ModbusState::ERROR;
          last_state_change = millis();
          break;
        }

        bool match = response[0] == this->current_request_.address &&
                     response[1] == this->current_request_.function &&
                     response[2] == ((this->current_request_.start_register >> 8) & 0xFF) &&
                     response[3] == (this->current_request_.start_register & 0xFF) &&
                     response[4] == ((this->current_request_.length_or_value >> 8) & 0xFF) &&
                     response[5] == (this->current_request_.length_or_value & 0xFF);

        if (!match) {
          ESP_LOGW(TAG, "[FSM] Echoed write mismatch in 0x06 response");
          this->fsm_state_ = ModbusState::ERROR;
          last_state_change = millis();
          break;
        }

        ESP_LOGD(TAG, "[FSM] Write confirmed: reg 0x%04X = 0x%04X",
                 this->current_request_.start_register,
                 this->current_request_.length_or_value);
      }

      // Success: invoke callback and return to IDLE
      if (this->current_request_.callback)
        this->current_request_.callback(true, result);

      this->fsm_state_ = ModbusState::IDLE;
      last_state_change = millis();
      break;
    }

    // --- FSM State: ERROR ---
    // Handle failures; retry if retries remain, or fail permanently
    case ModbusState::ERROR:
      ESP_LOGW(TAG, "[FSM] Error during request");

      if (this->current_request_.retries_left > 0) {
        this->current_request_.retries_left--;
        ESP_LOGW(TAG, "[FSM] Retrying request (retries left: %d)", this->current_request_.retries_left);
        this->fsm_state_ = ModbusState::SEND_REQUEST;
        last_state_change = millis();
      } else {
        ESP_LOGE(TAG, "[FSM] Retries exhausted, reporting failure");
        if (this->current_request_.callback)
          this->current_request_.callback(false, {});
        this->fsm_state_ = ModbusState::IDLE;
        last_state_change = millis();
        this->failed_requests_++;  // Increment failed requests
      }
      break;
  }
}

// --- Read UART RX buffer ---
bool ModbusAsciiHandler::read_available_() {
  if (this->uart_ == nullptr)
    return false;

  while (this->uart_->available()) {
    uint8_t byte;
    if (this->uart_->read_byte(&byte)) {
      // Ignore any leading noise until a Modbus ASCII frame start marker is found.
      if (this->rx_buffer_.empty()) {
        if (byte != ':')
          continue;
        this->rx_buffer_.push_back(byte);
        continue;
      }

      // If a new start marker appears mid-stream, resynchronize to the newer frame.
      if (byte == ':') {
        this->rx_buffer_.clear();
        this->rx_buffer_.push_back(byte);
        continue;
      }

      this->rx_buffer_.push_back(byte);

      if (this->rx_buffer_.size() >= 2 &&
          this->rx_buffer_[this->rx_buffer_.size() - 2] == '\r' &&
          this->rx_buffer_.back() == '\n') {
        return true;
      }

      if (this->rx_buffer_.size() > MAX_FRAME_BUFFER_SIZE) {
        this->rx_buffer_.clear();
        break;
      }
    }
  }

  return false;
}

}  // namespace olimpia_bridge
}  // namespace esphome
