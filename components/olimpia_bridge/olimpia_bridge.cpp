#include <vector>
#include <utility>
#include "esphome.h"
#include "esphome/core/log.h"
#include "olimpia_bridge.h"
#include "olimpia_bridge_climate.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "orchestrator";

bool OlimpiaBridge::validate_handler() const {
  if (this->handler_ == nullptr) {
    ESP_LOGE(TAG, "No ModbusAsciiHandler configured");
    return false;
  }
  if (!this->handler_->is_ready()) {
    ESP_LOGE(TAG, "ModbusAsciiHandler not fully configured");
    return false;
  }
  return true;
}

// --- Component Setup ---
void OlimpiaBridge::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Olimpia Bridge");

  if (!this->validate_handler()) {
    this->mark_failed();
    return;
  }

  // Handler is already registered as a component via Python config
  ESP_LOGCONFIG(TAG, "ModbusAsciiHandler initialized");
  ESP_LOGCONFIG(TAG, "OlimpiaBridge setup complete");
}

// --- Periodic Update Cycle ---
void OlimpiaBridge::update() {
  ESP_LOGD(TAG, "Running periodic control cycle for all climates...");

  // Update and publish error ratio sensor value
  if (this->error_ratio_sensor_ && this->handler_) {
    float error_ratio = this->handler_->get_error_ratio();
    this->error_ratio_sensor_->publish_state(error_ratio);
  }
}

// --- Home Assistant Service: Read Register ---
void OlimpiaBridge::read_register(int address, int reg) {
  ESP_LOGI(TAG, "Reading register %d on address %d", reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "Modbus handler not initialized; read_register skipped");
    return;
  }

  this->handler_->read_register(
    static_cast<uint8_t>(address),
    static_cast<uint16_t>(reg),
    1,
    [address, reg](bool success, std::vector<uint16_t> response) {
      if (!success) {
        ESP_LOGW(TAG, "Read FAILED: addr %d reg %d", address, reg);
        return;
      }

      if (response.empty()) {
        ESP_LOGW(TAG, "Read OK but response empty: addr %d reg %d", address, reg);
        return;
      }

      uint16_t value = response[0];
      ESP_LOGI(TAG, "Read OK: addr %d reg %d → 0x%04X (%d)", address, reg, value, value);

      // Optional: handle value (e.g., update sensor, publish state)
    }
  );
}

// --- Home Assistant Service: Write Register ---
void OlimpiaBridge::write_register(int address, int reg, int value) {
  ESP_LOGI(TAG, "Writing value %d to register %d on address %d", value, reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "Modbus handler not initialized; write_register skipped");
    return;
  }

  this->handler_->write_register(
    static_cast<uint8_t>(address),
    static_cast<uint16_t>(reg),
    static_cast<uint16_t>(value),
    [address, reg, value](bool success, std::vector<uint16_t>) {
      if (!success) {
        ESP_LOGW(TAG, "Write FAILED: addr %d reg %d", address, reg);
        return;
      }

      ESP_LOGI(TAG, "Write OK: addr %d reg %d ← 0x%04X (%d)", address, reg, value, value);

      // Optional: confirm state, refresh read, etc.
    }
  );
}

// --- Home Assistant Service: Dump Configuration ---
void OlimpiaBridge::dump_configuration(int address) {
  uint8_t addr = static_cast<uint8_t>(address);  // Safely cast to uint8_t

  static uint16_t current_register = 0;
  static std::vector<std::pair<uint16_t, uint16_t>> results;  // (reg, value)

  if (current_register == 0) {
    results.clear();
    ESP_LOGI(TAG, "Started config dump for address %u in background, wait..", addr);
  }

  if (current_register > 255) {
    // All done — split log into chunks to avoid truncation
    constexpr size_t chunk_size = 30;
    size_t total = results.size();
    for (size_t i = 0; i < total; i += chunk_size) {
      std::string line;
      char header[32];
      snprintf(header, sizeof(header), "[0x%02X] DUMP:", addr);
      line += header;

      for (size_t j = i; j < i + chunk_size && j < total; ++j) {
        char buf[32];
        snprintf(buf, sizeof(buf), " R%03u=(%u)", results[j].first, results[j].second);
        line += buf;
      }

      ESP_LOGI(TAG, "%s", line.c_str());
    }

    current_register = 0;  // Reset for future use
    return;
  }

  // Read the current register
  uint16_t reg = current_register;
  this->handler_->read_register(addr, reg, 1, [this, addr, reg](bool ok, const std::vector<uint16_t> &data) {
    if (ok && !data.empty()) {
      results.emplace_back(reg, data[0]);
    } else {
      ESP_LOGW(TAG, "[0x%02X] Failed to read register %u", addr, reg);
      results.emplace_back(reg, 0xFFFF);  // Optionally flag failed reads
    }

    current_register++;

    // Continue after 30ms
    this->set_timeout("dump_config", 30, [this, addr]() {
      this->dump_configuration(static_cast<int>(addr));
    });
  });
}

// --- Climate Entity Registration ---
void OlimpiaBridge::add_climate(OlimpiaBridgeClimate *climate) {
  this->climates_.push_back(climate);
}

}  // namespace olimpia_bridge
}  // namespace esphome
