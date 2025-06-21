#include "olimpia_bridge_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

// --- Logging Tag ---
static const char *const TAG = "Climate";

// --- Setup ---
void OlimpiaBridgeClimate::setup() {
  ESP_LOGI(TAG, "[%s] Climate setup initialized.", this->get_name().c_str());
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

  this->handler_->read_register(this->address_, 101, [this](bool success, const std::vector<uint16_t> &values) {
    if (!success || values.empty()) {
      ESP_LOGW(TAG, "[%s] Failed to read register 101", this->get_name().c_str());
      return;
    }

    uint16_t reg = values[0];
    ParsedState parsed = parse_command_register(reg);
    this->update_state_from_parsed(parsed);
  });
}

// --- Register 1 (water temp) ---
void OlimpiaBridgeClimate::read_water_temperature() {
  if (this->handler_ == nullptr || this->water_temp_sensor_ == nullptr) return;

  this->handler_->read_register(this->address_, 1, [this](bool success, const std::vector<uint16_t> &values) {
    if (!success || values.empty()) {
      ESP_LOGW(TAG, "[%s] Failed to read water temperature", this->get_name().c_str());
      return;
    }

    float temp = values[0] * 0.1f;
    ESP_LOGI(TAG, "[%s] Water temperature: %.1f°C", this->get_name().c_str(), temp);
    this->water_temp_sensor_->publish_state(temp);
  });
}

// --- Periodic FSM control cycle ---
void OlimpiaBridgeClimate::control_cycle() {
  uint32_t now = millis();
  if (!this->boot_cycle_done_ || (now - this->last_update_time_ > 60000)) {
    this->write_control_registers_cycle();
    this->read_water_temperature();
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

  this->handler_->write_register(this->address_, 101, reg101, [this, reg102, reg103](bool ok1, const std::vector<uint16_t> &) {
    if (!ok1) return;

    this->handler_->write_register(this->address_, 102, reg102, [this, reg103](bool ok2, const std::vector<uint16_t> &) {
      if (!ok2) return;

      this->handler_->write_register(this->address_, 103, reg103, [this](bool ok3, const std::vector<uint16_t> &) {
        if (!ok3) return;
      });
    });
  });
}

// --- Update from parsed state (register 101) ---
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {
  this->on_ = parsed.on;
  this->mode_ = parsed.mode;
  this->fan_speed_ = parsed.fan_speed;

  if (!this->on_) this->mode = climate::CLIMATE_MODE_OFF;
  else {
    switch (this->mode_) {
      case Mode::HEATING: this->mode = climate::CLIMATE_MODE_HEAT; break;
      case Mode::COOLING: this->mode = climate::CLIMATE_MODE_COOL; break;
      case Mode::AUTO:    this->mode = climate::CLIMATE_MODE_AUTO; break;
      default:            this->mode = climate::CLIMATE_MODE_OFF; break;
    }
  }

  switch (this->fan_speed_) {
    case FanSpeed::AUTO:  this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case FanSpeed::MIN:   this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case FanSpeed::NIGHT: this->fan_mode = climate::CLIMATE_FAN_QUIET; break;
    case FanSpeed::MAX:   this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default:              this->fan_mode.reset(); break;
  }

  this->publish_state();
}

// --- Register 101 encoder ---
uint16_t OlimpiaBridgeClimate::get_status_register() {
  uint16_t reg = 0;
  reg |= static_cast<uint8_t>(this->fan_speed_) & 0x07;
  if (!this->on_) reg |= (1 << 7);

  switch (this->mode_) {
    case Mode::AUTO:    reg |= (0b00 << 13); break;
    case Mode::HEATING: reg |= (0b01 << 13); break;
    case Mode::COOLING: reg |= (0b10 << 13); break;
    default:            reg |= (0b00 << 13); break;
  }
  return reg;
}

// --- External ambient temperature input (used for register 103) ---
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  if (std::isnan(temp)) return;
  this->external_ambient_temperature_ = temp;
  this->current_temperature_ = temp;
  this->has_received_external_temp_ = true;
  this->publish_state();
}

// --- Register 101 parser ---
ParsedState parse_command_register(uint16_t reg) {
  ParsedState state;
  state.fan_speed_raw = reg & 0x07;
  switch (state.fan_speed_raw) {
    case 0b000: state.fan_speed = FanSpeed::AUTO; break;
    case 0b001: state.fan_speed = FanSpeed::MIN; break;
    case 0b010: state.fan_speed = FanSpeed::NIGHT; break;
    case 0b011: state.fan_speed = FanSpeed::MAX; break;
    default:    state.fan_speed = FanSpeed::UNKNOWN; break;
  }

  state.on = !((reg >> 7) & 0x01);
  state.cp = (reg >> 12) & 0x01;
  uint8_t ei = (reg >> 13) & 0x03;
  switch (ei) {
    case 0b00: state.mode = Mode::AUTO; break;
    case 0b01: state.mode = Mode::HEATING; break;
    case 0b10: state.mode = Mode::COOLING; break;
    default:   state.mode = Mode::UNKNOWN; break;
  }

  return state;
}

}  // namespace olimpia_bridge
}  // namespace esphome
