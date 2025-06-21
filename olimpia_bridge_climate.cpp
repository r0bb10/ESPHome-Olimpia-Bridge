#include "olimpia_bridge_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "Climate";

// --- Register 101 parser (bit-mapped status register) ---
ParsedState OlimpiaBridgeClimate::parse_command_register(uint16_t reg) {
  ParsedState state;

  // --- Bits 0–2: PRG fan speed ---
  // 000 = Auto, 001 = Min, 010 = Night, 011 = Max, others = Unknown
  state.fan_speed_raw = reg & 0x07;
  switch (state.fan_speed_raw) {
    case 0b000: state.fan_speed = FanSpeed::AUTO; break;
    case 0b001: state.fan_speed = FanSpeed::MIN; break;
    case 0b010: state.fan_speed = FanSpeed::NIGHT; break;
    case 0b011: state.fan_speed = FanSpeed::MAX; break;
    default:    state.fan_speed = FanSpeed::UNKNOWN; break;
  }

  // --- Bit 7: STBY (working condition) ---
  // 0 = Activated (ON), 1 = Standby (OFF)
  state.on = ((reg & (1 << 7)) == 0);

  // --- Bit 12: CP (contact presence) ---
  // Ignored in logic (only logged optionally)
  state.cp = (reg & (1 << 12)) != 0;

  // --- Bits 13–14: EI mode of functioning ---
  // 00 = Auto, 01 = Heating, 10 = Cooling, others = Unknown
  switch ((reg >> 13) & 0b11) {
    case 0b00: state.mode = Mode::AUTO; break;
    case 0b01: state.mode = Mode::HEATING; break;
    case 0b10: state.mode = Mode::COOLING; break;
    default:   state.mode = Mode::UNKNOWN; break;
  }
  return state;
}

// --- Register 101 encoder (bit-mapped control register) ---
uint16_t OlimpiaBridgeClimate::build_command_register(bool on, Mode mode, FanSpeed fan_speed) {
  uint16_t reg = 0;

  // --- Bits 0–2: PRG fan speed ---
  // 000 = Auto, 001 = Min, 010 = Night, 011 = Max
  reg |= static_cast<uint8_t>(fan_speed) & 0x07;

  // --- Bit 7: STBY (working condition) ---
  // 0 = Activated, 1 = Standby (OFF)
  if (!on)
    reg |= (1 << 7);

  // --- Bit 12: CP presence contact ---
  reg &= ~(1 << 12);  // Ensure CP is always 0

  // --- Bits 13–14: EI mode of functioning ---
  // 10 = Cooling, 01 = Heating, 00 = Auto
  switch (mode) {
    case Mode::AUTO:
      reg |= (0b00 << 13);
      break;
    case Mode::HEATING:
      reg |= (0b01 << 13);
      break;
    case Mode::COOLING:
      reg |= (0b10 << 13);
      break;
    default:
      reg |= (0b00 << 13);  // Fallback to AUTO
      break;
  }
  return reg;
}

// --- Setup ---
void OlimpiaBridgeClimate::setup() {
  ESP_LOGI(TAG, "[%s] Climate setup initialized.", this->get_name().c_str());

  this->refresh_from_register_101();  // Force initial status read
}

// --- Traits ---
climate::ClimateTraits OlimpiaBridgeClimate::traits() {
  climate::ClimateTraits traits;
  traits.set_supports_current_temperature(true);
  traits.set_supports_action(true);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_AUTO,
  });
  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_QUIET,
    climate::CLIMATE_FAN_HIGH,
  });
  traits.set_visual_temperature_step(0.5f);
  return traits;
}

// --- HA control callback ---
void OlimpiaBridgeClimate::control(const climate::ClimateCall &call) {
  bool state_changed = false;

  if (call.get_target_temperature().has_value()) {
    this->target_temperature_ = *call.get_target_temperature();
    ESP_LOGI(TAG, "[%s] Target temperature set to %.1f°C", this->get_name().c_str(), this->target_temperature_);
    state_changed = true;
  }

  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    switch (new_mode) {
      case climate::CLIMATE_MODE_OFF:
        this->on_ = false;
        this->mode_ = Mode::AUTO;
        break;
      case climate::CLIMATE_MODE_COOL:
        this->on_ = true;
        this->mode_ = Mode::COOLING;
        break;
      case climate::CLIMATE_MODE_HEAT:
        this->on_ = true;
        this->mode_ = Mode::HEATING;
        break;
      case climate::CLIMATE_MODE_AUTO:
        this->on_ = true;
        this->mode_ = Mode::AUTO;
        break;
      default:
        break;
    }
    state_changed = true;
  }

  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    if (new_fan == climate::CLIMATE_FAN_AUTO) this->fan_speed_ = FanSpeed::AUTO;
    else if (new_fan == climate::CLIMATE_FAN_LOW) this->fan_speed_ = FanSpeed::MIN;
    else if (new_fan == climate::CLIMATE_FAN_QUIET) this->fan_speed_ = FanSpeed::NIGHT;
    else if (new_fan == climate::CLIMATE_FAN_HIGH) this->fan_speed_ = FanSpeed::MAX;
    state_changed = true;
  }

  if (state_changed) {
    this->write_control_registers_cycle();
    this->refresh_from_register_101();
  }
}

// --- Async Modbus read (register 101) ---
void OlimpiaBridgeClimate::refresh_from_register_101() {
  if (this->handler_ == nullptr) return;

  ESP_LOGI(TAG, "[%s] Refreshing register 101...", this->get_name().c_str());

  this->handler_->read_register(this->address_, 101, 1, [this](bool success, const std::vector<uint16_t> &data) {
    if (!success || data.empty()) {
      ESP_LOGW(TAG, "[%s] Failed to read register 101", this->get_name().c_str());
      return;
    }

    uint16_t reg = data[0];
    ESP_LOGI(TAG, "[%s] Read register 101 raw: 0x%04X (%d)", this->get_name().c_str(), reg, reg);

    ParsedState parsed = this->parse_command_register(reg);

    ESP_LOGI(TAG, "[%s] Decoded 101 → on=%d mode=%d fan=%d cp=%d", this->get_name().c_str(),
             parsed.on, static_cast<int>(parsed.mode),
             static_cast<int>(parsed.fan_speed), parsed.cp);

    this->update_state_from_parsed(parsed);
  });
}

// --- Register 1 (water temp) ---
void OlimpiaBridgeClimate::read_water_temperature() {
  if (this->handler_ == nullptr || this->water_temp_sensor_ == nullptr) return;

  this->handler_->read_register(this->address_, 1, 1, [this](bool success, const std::vector<uint16_t> &data) {
    if (!success || data.empty()) {
      ESP_LOGW(TAG, "[%s] Failed to read water temperature", this->get_name().c_str());
      return;
    }

    float temp = data[0] * 0.1f;
    ESP_LOGI(TAG, "[%s] Water temperature: %.1f°C", this->get_name().c_str(), temp);
    this->water_temp_sensor_->publish_state(temp);
  });
}

// --- Periodic FSM control cycle ---
void OlimpiaBridgeClimate::control_cycle() {
  const uint32_t now = millis();

  // Update every 60s or on first boot
  if (!this->boot_cycle_done_ || (now - this->last_update_time_ > 60000)) {
    ESP_LOGD(TAG, "[%s] Starting control cycle", this->get_name().c_str());

    // --- Push control values to registers 101/102 ---
    this->write_control_registers_cycle();

    // --- Refresh current state from register 101 ---
    this->refresh_from_register_101();

    // --- Read water temperature (register 1) ---
    this->read_water_temperature();

    // --- Always push last known external ambient temp to register 103 ---
    if (!std::isnan(this->external_ambient_temperature_)) {
      uint16_t reg103 = static_cast<uint16_t>(this->external_ambient_temperature_ * 10);
      this->handler_->write_register(this->address_, 103, reg103, [this](bool success, const std::vector<uint16_t> &) {
        if (success) {
          ESP_LOGD(TAG, "[%s] Refreshed register 103 with external temp: %.1f°C", this->get_name().c_str(), this->external_ambient_temperature_);
        } else {
          ESP_LOGW(TAG, "[%s] Failed to refresh register 103 (external temp)", this->get_name().c_str());
        }
      });
    } else {
      ESP_LOGW(TAG, "[%s] No external temperature available for register 103", this->get_name().c_str());
    }

    // --- Force state publish every cycle
    ESP_LOGD(TAG, "[%s] Periodic control cycle triggered", this->get_name().c_str());
    this->publish_state();

    // --- Update timestamps ---
    this->last_update_time_ = now;
    this->boot_cycle_done_ = true;
  }
}

// --- FSM write 101 → 102 → 103 ---
void OlimpiaBridgeClimate::write_control_registers_cycle() {
  if (this->handler_ == nullptr) return;

  uint16_t reg101 = this->get_status_register();
  uint16_t reg102 = static_cast<uint16_t>(this->target_temperature_ * 10);
  uint16_t reg103 = std::isnan(this->external_ambient_temperature_) ? 0 : static_cast<uint16_t>(this->external_ambient_temperature_ * 10);

  ESP_LOGI(TAG, "[%s] Writing 101: 0x%04X | 102: %.1f°C | 103: %.1f°C",
           this->get_name().c_str(), reg101,
           this->target_temperature_, this->external_ambient_temperature_);

  this->handler_->write_register(this->address_, 101, reg101, [this, reg102, reg103](bool ok1, const std::vector<uint16_t> &) {
    if (!ok1) {
      ESP_LOGW(TAG, "[%s] Failed to write register 101", this->get_name().c_str());
      return;
    }

    this->handler_->write_register(this->address_, 102, reg102, [this, reg103](bool ok2, const std::vector<uint16_t> &) {
      if (!ok2) {
        ESP_LOGW(TAG, "[%s] Failed to write register 102", this->get_name().c_str());
        return;
      }

      this->handler_->write_register(this->address_, 103, reg103, [this](bool ok3, const std::vector<uint16_t> &) {
        if (!ok3) {
          ESP_LOGW(TAG, "[%s] Failed to write register 103", this->get_name().c_str());
          return;
        }
      });
    });
  });
}


// --- Update from parsed state (register 101) ---
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {
  // --- Copy state from parsed register 101
  this->on_ = parsed.on;
  this->mode_ = parsed.mode;
  this->fan_speed_ = parsed.fan_speed;

  // --- Map internal state to HA climate mode
  if (!this->on_) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (this->mode_) {
      case Mode::HEATING:
        this->mode = climate::CLIMATE_MODE_HEAT;
        break;
      case Mode::COOLING:
        this->mode = climate::CLIMATE_MODE_COOL;
        break;
      case Mode::AUTO:
        this->mode = climate::CLIMATE_MODE_AUTO;
        break;
      default:
        this->mode = climate::CLIMATE_MODE_OFF;
        break;
    }
  }

  // --- Map fan speed to HA fan mode
  switch (this->fan_speed_) {
    case FanSpeed::AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case FanSpeed::MIN:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case FanSpeed::NIGHT:
      this->fan_mode = climate::CLIMATE_FAN_QUIET;
      break;
    case FanSpeed::MAX:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      this->fan_mode.reset();
      break;
  }

  // --- Push temperatures to ESPHome climate state
  this->current_temperature = this->external_ambient_temperature_;
  this->target_temperature = this->target_temperature_;  // use internal as reference for now

  // --- Log state for debugging
  ESP_LOGI(TAG, "[%s] Updated state from reg101: ON=%d MODE=%d FAN=%d → current=%.1f°C target=%.1f°C",
           this->get_name().c_str(), this->on_, static_cast<int>(this->mode_),
           static_cast<int>(this->fan_speed_), this->current_temperature, this->target_temperature);

  // --- Push state to HA
  this->publish_state();
}

uint16_t OlimpiaBridgeClimate::get_status_register() {
  uint16_t reg = 0;

  // Encode fan speed (bits 0–2)
  reg |= static_cast<uint8_t>(this->fan_speed_) & 0x07;

  // Encode power status (bit 7): 0 = ON, 1 = OFF
  if (!this->on_) reg |= (1 << 7);

  // Encode mode (bits 13–14)
  switch (this->mode_) {
    case Mode::AUTO:    reg |= (0b00 << 13); break;
    case Mode::HEATING: reg |= (0b01 << 13); break;
    case Mode::COOLING: reg |= (0b10 << 13); break;
    default:            reg |= (0b00 << 13); break;
  }

  ESP_LOGI(TAG, "[%s] Composed 101 ← 0x%04X (on=%d mode=%d fan=%d)",
           this->get_name().c_str(), reg, this->on_, static_cast<int>(this->mode_), static_cast<int>(this->fan_speed_));

  return reg;
}

// --- External ambient temperature input (used for register 103) ---
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  if (std::isnan(temp))
    return;

  this->external_ambient_temperature_ = temp;
  this->current_temperature_ = temp;
  this->has_received_external_temp_ = true;

  ESP_LOGI(TAG, "[%s] Received new external ambient temperature: %.1f°C", this->get_name().c_str(), temp);

  this->publish_state();  // Update climate UI

  // Optional: write to register 103 immediately
  this->handler_->write_register(this->address_, 103, static_cast<uint16_t>(temp * 10), [this](bool success, const std::vector<uint16_t> &) {
    if (!success) {
      ESP_LOGW(TAG, "[%s] Failed to write register 103 (external temp)", this->get_name().c_str());
      return;
    }
    ESP_LOGI(TAG, "[%s] Wrote external temp %.1f°C to register 103", this->get_name().c_str(), this->external_ambient_temperature_);
  });
}

}  // namespace olimpia_bridge
}  // namespace esphome
