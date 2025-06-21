// --- MODBUS ASCII HANDLER IMPLEMENTATION ---
#include "modbus_ascii_handler.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "modbus_ascii";

// --- Direction control ---
void ModbusAsciiHandler::set_direction(bool transmit) {
  if (this->re_pin_ != nullptr)
    this->re_pin_->digital_write(transmit);  // RE low to enable RX, high to disable
  if (this->de_pin_ != nullptr)
    this->de_pin_->digital_write(transmit);   // DE high to enable TX, low to disable
}

// --- LRC Checksum ---
uint8_t ModbusAsciiHandler::compute_lrc(const std::vector<uint8_t> &data) {
  uint8_t sum = 0;
  for (uint8_t b : data)
    sum += b;
  return static_cast<uint8_t>(-sum);  // two's complement, same as ~sum + 1
}

// --- Hex helper functions for speed and clarity ---
static constexpr char HEX_CHARS[] = "0123456789ABCDEF";

inline uint8_t hex_char_to_val(char c) {
  // Fast hex conversion, assuming valid ASCII input
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;  // invalid
}

// --- ASCII Encoding ---
std::string ModbusAsciiHandler::encode_ascii_frame(const std::vector<uint8_t> &data) {
  // Reserve exact size upfront for performance: 
  // 1 start + 2 chars per byte + 2 chars LRC + 2 for CRLF
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

  ESP_LOGD(TAG, "[Modbus] Encoded ASCII frame: %s", result.c_str());

  return result;
}

// --- ASCII Decoding ---
bool ModbusAsciiHandler::decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data) {
  if (frame.size() < 7 || frame[0] != ':' || frame.substr(frame.size() - 2) != "\r\n") {
    ESP_LOGW(TAG, "[Modbus] Invalid frame start/end");
    return false;
  }

  size_t payload_len = frame.size() - 3;  // exclude ':' and "\r\n"
  if (payload_len % 2 != 0) {
    ESP_LOGW(TAG, "[Modbus] Invalid frame length");
    return false;
  }

  data.clear();
  data.reserve(payload_len / 2 - 1);  // minus 1 for LRC byte

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

  // Parse received LRC from last two hex chars before \r\n
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

// --- Public Modbus API (async) ---
void ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint16_t count,
                                       std::function<void(bool success, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x03;  // Read Holding Registers
  req.start_register = reg;
  req.length_or_value = count;
  req.is_write = false;
  req.callback = std::move(callback);

  this->add_request(req);
}

void ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value,
                                        std::function<void(bool success, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x06;  // Write Single Register
  req.start_register = reg;
  req.length_or_value = value;
  req.is_write = true;
  req.callback = std::move(callback);

  this->add_request(req);
}

void ModbusAsciiHandler::write_byte(uint8_t byte) {
  if (this->uart_ != nullptr)
    this->uart_->write_byte(byte);
}

// --- FSM: Frame Builder ---
std::vector<uint8_t> ModbusAsciiHandler::build_request_frame_ascii_(const std::vector<uint8_t> &data) {
  std::string ascii = this->encode_ascii_frame(data);
  return std::vector<uint8_t>(ascii.begin(), ascii.end());
}

// --- FSM: Request Queue ---
void ModbusAsciiHandler::add_request(ModbusRequest request) {
  this->request_queue_.push(std::move(request));
  ESP_LOGD(TAG, "[FSM] Request enqueued (fn=0x%02X reg=0x%04X)", request.function, request.start_register);
}

// --- FSM: Loop ---
void ModbusAsciiHandler::loop() {
  switch (this->fsm_state_) {
    case ModbusState::IDLE:
      if (!this->request_queue_.empty()) {
        this->current_request_ = this->request_queue_.front();
        this->request_queue_.pop();
        ESP_LOGD(TAG, "[FSM] Transition: IDLE → SEND_REQUEST");
        this->fsm_state_ = ModbusState::SEND_REQUEST;
      }
      break;

    case ModbusState::SEND_REQUEST:
      {
        std::vector<uint8_t> request_pdu;

        request_pdu.push_back(this->current_request_.address);
        request_pdu.push_back(this->current_request_.function);
        request_pdu.push_back((this->current_request_.start_register >> 8) & 0xFF);
        request_pdu.push_back(this->current_request_.start_register & 0xFF);

        uint16_t val = this->current_request_.length_or_value;
        request_pdu.push_back((val >> 8) & 0xFF);
        request_pdu.push_back(val & 0xFF);

        std::string frame_ascii = this->encode_ascii_frame(request_pdu);  // ← your working encoder

        ESP_LOGD(TAG, "[FSM] TX ASCII Frame: %s", frame_ascii.c_str());

        this->set_direction(true);
        delay(2);

        this->uart_->write_str(frame_ascii.c_str());
        this->uart_->flush();
        delay(5);

        this->set_direction(false);

        this->fsm_start_time_ = millis();
        this->fsm_state_ = ModbusState::WAIT_RESPONSE;
      }
      break;

    case ModbusState::WAIT_RESPONSE:
      if (this->read_available_()) {
        ESP_LOGD(TAG, "[FSM] Response ready, processing");
        this->fsm_state_ = ModbusState::PROCESS_RESPONSE;
      } else if (millis() - this->fsm_start_time_ > fsm_timeout_ms_) {
        ESP_LOGW(TAG, "[FSM] Timeout waiting for response");
        this->fsm_state_ = ModbusState::ERROR;
      }
      break;

    case ModbusState::PROCESS_RESPONSE: {
      // --- FSM: PROCESS_RESPONSE ---
      ESP_LOGD(TAG, "[FSM] Processing response");

      // Log raw RX buffer as hex
      std::string hex_dump;
      for (uint8_t byte : this->rx_buffer_) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", byte);
        hex_dump += buf;
      }
      ESP_LOGV(TAG, "[FSM] Raw RX buffer: %s", hex_dump.c_str());

      // Convert vector to string for decoding
      std::string rx_string(this->rx_buffer_.begin(), this->rx_buffer_.end());
      std::vector<uint8_t> response;
      bool decoded = this->decode_ascii_frame(rx_string, response);
      this->rx_buffer_.clear();

      if (!decoded) {
        ESP_LOGW(TAG, "[FSM] Invalid ASCII frame — decode failed");
        this->fsm_state_ = ModbusState::ERROR;
        break;
      }

      // Validate address
      if (response.size() < 2 || response[0] != this->current_request_.address) {
        ESP_LOGW(TAG, "[FSM] Address mismatch (got=0x%02X, expected=0x%02X)",
                response.empty() ? 0xFF : response[0], this->current_request_.address);
        this->fsm_state_ = ModbusState::ERROR;
        break;
      }

      std::vector<uint16_t> result;

      if (this->current_request_.function == 0x03 || this->current_request_.function == 0x04) {
        if (response.size() < 3) {
          ESP_LOGW(TAG, "[FSM] Response too short");
          this->fsm_state_ = ModbusState::ERROR;
          break;
        }

        uint8_t byte_count = response[2];
        if (response.size() < 3 + byte_count) {
          ESP_LOGW(TAG, "[FSM] Incomplete data payload (expected %d bytes, got %d)",
                  byte_count, static_cast<int>(response.size()) - 3);
          this->fsm_state_ = ModbusState::ERROR;
          break;
        }

        for (size_t i = 0; i + 1 < byte_count; i += 2) {
          uint16_t val = (response[3 + i] << 8) | response[3 + i + 1];
          result.push_back(val);
          ESP_LOGV(TAG, "[FSM] Parsed reg[%zu] = 0x%04X", i / 2, val);
        }
      }

      // Notify upper layer
      if (this->current_request_.callback)
        this->current_request_.callback(true, result);

      this->fsm_state_ = ModbusState::IDLE;
      break;
    }

    case ModbusState::ERROR:
      ESP_LOGE(TAG, "[FSM] Error during request");
      if (this->current_request_.callback)
        this->current_request_.callback(false, {});
      this->fsm_state_ = ModbusState::IDLE;
      break;
  }
}

bool ModbusAsciiHandler::read_available_() {
  if (!this->uart_ || !this->uart_->available())
    return false;

  while (this->uart_->available()) {
    uint8_t byte;
    if (this->uart_->read_byte(&byte)) {
      this->rx_buffer_.push_back(byte);

      // Log every byte as it comes in (ASCII + HEX)
      ESP_LOGVV(TAG, "[FSM] RX byte: 0x%02X (%c)", byte, (byte >= 32 && byte <= 126) ? byte : '.');

      // Check for end of frame
      if (this->rx_buffer_.size() >= 2 &&
          this->rx_buffer_[this->rx_buffer_.size() - 2] == '\r' &&
          this->rx_buffer_.back() == '\n') {
        ESP_LOGV(TAG, "[FSM] Detected CRLF → end of frame");
        return true;
      }

      // Safety: prevent overrun
      if (this->rx_buffer_.size() > 128) {
        ESP_LOGW(TAG, "[FSM] RX overflow, buffer cleared");
        this->rx_buffer_.clear();
        break;
      }
    }
  }

  return false;
}

}  // namespace olimpia_bridge
}  // namespace esphome
