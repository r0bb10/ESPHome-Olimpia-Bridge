#include "modbus_ascii_handler.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "olimpia_bridge.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "modbus_ascii_handler";

// -------------------- RE/DE Direction Handling --------------------

void ModbusAsciiHandler::set_direction(bool transmit) {
  if (this->direction_pin_ != nullptr) {
    // Single direction pin controls both TX and RX
    this->direction_pin_->digital_write(transmit);
  } else {
    // Dual-pin mode: DE for TX control, RE always LOW to enable receiver
    if (this->de_pin_ != nullptr)
      this->de_pin_->digital_write(transmit);   // HIGH = TX enable

    if (this->re_pin_ != nullptr)
      this->re_pin_->digital_write(false);      // Always LOW = RX enabled
  }
}

// -------------------- LRC Checksum --------------------

uint8_t ModbusAsciiHandler::lrc(const std::vector<uint8_t> &data) {
  uint8_t sum = 0;
  for (auto b : data)
    sum += b;
  return (-sum) & 0xFF;
}

// -------------------- Encode Request Frame --------------------

std::string ModbusAsciiHandler::encode_ascii_frame(const std::vector<uint8_t> &data) {
  std::string result = ":";

  // Encode each data byte as two uppercase hex characters
  for (uint8_t byte : data) {
    char hex_byte[3];
    snprintf(hex_byte, sizeof(hex_byte), "%02X", byte);
    result += hex_byte;
  }

  // Compute LRC and append it
  uint8_t checksum = lrc(data);
  char lrc_buf[3];
  snprintf(lrc_buf, sizeof(lrc_buf), "%02X", checksum);
  result += lrc_buf;

  // End-of-frame markers
  result += "\r\n";

  // Optional debug log
  ESP_LOGV(TAG, "Encoded Modbus ASCII frame: '%s'", result.c_str());

  return result;
}

// -------------------- Decode Response Frame --------------------

bool parse_hex_byte(const std::string &s, uint8_t &out) {
  if (s.length() != 2) return false;
  char *end = nullptr;
  long val = strtol(s.c_str(), &end, 16);
  if (end == nullptr || *end != '\0' || val < 0 || val > 255)
    return false;
  out = static_cast<uint8_t>(val);
  return true;
}

bool ModbusAsciiHandler::decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data) {
  // Check basic structure
  if (frame.length() < 7 || frame.front() != ':' || !str_endswith(frame, "\r\n")) {
    ESP_LOGW(TAG, "Invalid frame format: too short or missing start/end markers: '%s'", frame.c_str());
    return false;
  }

  // Strip leading ':' and trailing "\r\n"
  std::string payload = frame.substr(1, frame.length() - 3);

  // Check that the payload length is even
  if (payload.length() % 2 != 0) {
    ESP_LOGW(TAG, "Invalid payload length (must be even): '%s'", payload.c_str());
    return false;
  }

  size_t byte_count = payload.length() / 2;
  if (byte_count < 2) {
    ESP_LOGW(TAG, "Payload too short to contain data and LRC: '%s'", payload.c_str());
    return false;
  }

  data.clear();

  // Parse all data bytes except the last two characters (LRC)
  for (size_t i = 0; i < byte_count - 1; ++i) {
    std::string byte_str = payload.substr(i * 2, 2);
    char *end = nullptr;
    uint8_t byte;
    if (!parse_hex_byte(byte_str, byte)) {
      ESP_LOGW(TAG, "Invalid hex byte '%s' at index %d in payload '%s'", byte_str.c_str(), static_cast<int>(i), payload.c_str());
      return false;
    }
    data.push_back(byte);
  }

  // Decode LRC
  std::string lrc_str = payload.substr((byte_count - 1) * 2, 2);
  int lrc_val = strtol(lrc_str.c_str(), nullptr, 16);
  uint8_t computed_lrc = lrc(data);

  if (computed_lrc != static_cast<uint8_t>(lrc_val)) {
    ESP_LOGW(TAG, "LRC mismatch: expected 0x%02X, received 0x%02X", computed_lrc, lrc_val);
    return false;
  }

  return true;
}

// -------------------- Full Modbus Transaction --------------------

bool ModbusAsciiHandler::send_and_receive(const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  constexpr size_t MAX_FRAME_SIZE = 256;

  ESP_LOGW(TAG, "send_and_receive() entered");

  // Log request content and size								 
  ESP_LOGW(TAG, "request size: %d bytes", request.size());
  for (size_t i = 0; i < request.size(); i++) {
    ESP_LOGW(TAG, "  - Byte[%zu] = 0x%02X", i, request[i]);
  }

  if (!this->uart_)
    return false;

  // Clear leftover bytes from RX buffer										
  int flushed = 0;
  while (this->uart_->available()) {
    uint8_t dummy;
    this->uart_->read_byte(&dummy);
    flushed++;
  }
  if (flushed > 0) {
    ESP_LOGW(TAG, "Flushed %d stale bytes from UART RX buffer before TX", flushed);
  }

  std::string frame = encode_ascii_frame(request);
  ESP_LOGW(TAG, "encode_ascii_frame returned %s bytes", frame.length() > 0 ? "non-zero" : "ZERO");

  // Check if the frame is empty
  if (frame.empty()) {
    ESP_LOGW(TAG, "Encoded frame is empty — nothing will be sent!");
  } else {
    ESP_LOGW(TAG, "TX frame prepared: %s", frame.c_str());
  }

  // --- Enable TX ---
  this->set_direction(true);  // TX mode
  delayMicroseconds(200);

											 

  this->uart_->write_str(frame.c_str());
  ESP_LOGW(TAG, "Writing to UART: %s", frame.c_str());
  this->uart_->flush();

  delayMicroseconds(2000);  // turnaround time

  // --- Enable RX ---
  this->set_direction(false);  // RX mode

  std::string buffer;
  uint32_t start_time = millis();

  while (millis() - start_time < 500) {
    uint8_t byte;

  // Increase read window to 500ms to match device latency														  
    while (this->uart_->read_byte(&byte)) {
      char c = static_cast<char>(byte);
      buffer += c;

    // Prevent runaway memory usage								   
      if (buffer.length() > MAX_FRAME_SIZE) {
        ESP_LOGW(TAG, "Received frame too long (%d bytes). Aborting read.", buffer.length());
        break;
      }

    // Debug each character received									
      ESP_LOGD(TAG, "UART char: 0x%02X '%c'", byte, isprint(c) ? c : '.');

      if (c == '\n' && buffer.length() >= 2 && buffer[buffer.length() - 2] == '\r') {
        ESP_LOGD(TAG, "Frame terminator detected");
        goto decode;
      }
    }
   

    delay(1);  // Avoid tight loop
  }

decode:
  ESP_LOGD(TAG, "RX raw frame: '%s'", buffer.c_str());

  // Attempt to decode Modbus ASCII response											
  bool ok = decode_ascii_frame(buffer, response);
  if (!ok) {
    ESP_LOGW(TAG, "Failed to decode response: '%s'", buffer.c_str());
  }

  // Track success/failure using OlimpiaBridge											  
  if (this->bridge_ != nullptr) {
    this->bridge_->increment_modbus_request(ok);
  }

  return ok;
}

// -------------------- Read Holding Register --------------------

bool ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint16_t *out) {
  std::vector<uint8_t> request = {
      address, 0x03,
      static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
      0x00, 0x01
  };
  std::vector<uint8_t> response;

  // First attempt
  if (!send_and_receive(request, response) || response.size() < 7) {
    // Retry once after a short delay
    ESP_LOGW(TAG, "Read failed for register 0x%04X on address 0x%02X. Retrying once...", reg, address);
    delay(10);
    if (!send_and_receive(request, response) || response.size() < 7)
      return false;
  }

  // Decode register value from response
  *out = (response[3] << 8) | response[4];
  return true;
}

// -------------------- Write Single Holding Register --------------------

bool ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value) {
  std::vector<uint8_t> request = {
      address, 0x06,
      static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  std::vector<uint8_t> response;
  return send_and_receive(request, response);
}

}  // namespace olimpia_bridge
}  // namespace esphome
