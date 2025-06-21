// --- OLIMPIA BRIDGE COMPONENT IMPLEMENTATION ---
#include "olimpia_bridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "olimpia_bridge";

void OlimpiaBridge::setup() {
  ESP_LOGI(TAG, "Setting up Olimpia Bridge");

  // Set RE and DE pins (no direction pin check)
  if (this->re_pin_ != nullptr && this->de_pin_ != nullptr) {
    this->re_pin_->setup();
    this->de_pin_->setup();
    this->re_pin_->digital_write(false);  // RX mode
    this->de_pin_->digital_write(false);  // RX mode
  } else {
    ESP_LOGE(TAG, "No RE/DE pair configured.");
    this->mark_failed();
    return;
  }

  // Set UART for handler
  if (this->handler_) {
    this->handler_->set_uart(this->uart_);
    this->handler_->set_re_pin(this->re_pin_);
    this->handler_->set_de_pin(this->de_pin_);
  }

  // Register API services
  this->register_service(&OlimpiaBridge::read_register,
                         "read_register",
                         {"address", "register"});
  this->register_service(&OlimpiaBridge::write_register,
                           "write_register",
                           {"address", "register", "value"});

  ESP_LOGI(TAG, "OlimpiaBridge setup complete");
}

void OlimpiaBridge::update() {
  // Nothing to poll by default – could trigger automatic register reads here if needed
}

void OlimpiaBridge::read_register(int address, int reg) {
  ESP_LOGI(TAG, "[Service] Reading register %d on address %d", reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "[Service] Modbus handler not initialized; read_register skipped");
    return;
  }

  std::vector<uint16_t> response;

  if (this->handler_->read_register(static_cast<uint8_t>(address), static_cast<uint16_t>(reg), 1, response)) {
    if (!response.empty()) {
      uint16_t value = response[0];
      ESP_LOGI(TAG, "[Service] Read OK: addr %d reg %d → 0x%04X (%d)", address, reg, value, value);
    } else {
      ESP_LOGW(TAG, "[Service] Read OK but response empty: addr %d reg %d", address, reg);
    }
  } else {
    ESP_LOGW(TAG, "[Service] Read FAILED: addr %d reg %d", address, reg);
  }
}

void OlimpiaBridge::write_register(int address, int reg, int value) {
  ESP_LOGI(TAG, "[Service] Writing value %d to register %d on address %d", value, reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "[Service] Modbus handler not initialized; write_register skipped");
    return;
  }

  bool success = this->handler_->write_register(static_cast<uint8_t>(address),
                                                static_cast<uint16_t>(reg),
                                                static_cast<uint16_t>(value));
  if (success) {
    ESP_LOGI(TAG, "[Service] Write OK: addr %d reg %d ← 0x%04X (%d)", address, reg, value, value);
  } else {
    ESP_LOGW(TAG, "[Service] Write FAILED: addr %d reg %d", address, reg);
  }
}

}  // namespace olimpia_bridge
}  // namespace esphome
