#include "olimpia_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h" 
#include "esphome/components/api/api_pb2.h"
#include "esphome/components/api/user_services.h"
#include "esphome/components/olimpia_bridge/olimpia_bridge.h"

namespace esphome {

namespace api {

//template <>
//inline int get_execute_arg_value<int>(const ExecuteServiceArgument &arg) {
//  return arg.legacy_int;  // confirmed to exist in ESPHome 2025.5.2
//}

//template <>
//inline enums::ServiceArgType to_service_arg_type<int>() {
//  return enums::SERVICE_ARG_TYPE_INT;
//}

}  // namespace api

namespace olimpia_bridge {

static const char *const TAG = "olimpia_bridge";

// Constructs register 101 value from component state
uint16_t build_command_register(bool on, Mode mode, FanSpeed fan_speed) {
  uint16_t reg = 0;
  reg |= static_cast<uint8_t>(fan_speed) & 0x07;     // Bits 0–2: PRG

  if (!on) reg |= 1 << 7;                            // Bit 7: STBY = 1 if off

  // Clear bits 13–14
  reg &= ~(0b11 << 13);

  switch (mode) {
    case Mode::HEATING:
      reg |= (0b01 << 13);  // EI = 01
      break;
    case Mode::COOLING:
      reg |= (0b10 << 13);  // EI = 10
      break;
    case Mode::AUTO:
      reg |= (0b11 << 13);  // EI = 11
      break;
    default:
      // EI = 00 (OFF)
      break;
  }

  return reg;  // Reserved bits remain zero
}

// Parses register 101 into structured state
ParsedState parse_command_register(uint16_t reg) {
  ParsedState state;
  state.on = !(reg & (1 << 7));                      // STBY bit

  bool ei_0 = (reg >> 13) & 0x1;
  bool ei_1 = (reg >> 14) & 0x1;

  if (!ei_0 && !ei_1)
    state.mode = Mode::OFF;
  else if (ei_0 && !ei_1)
    state.mode = Mode::HEATING;
  else if (!ei_0 && ei_1)
    state.mode = Mode::COOLING;
  else  // ei_0 && ei_1
    state.mode = Mode::AUTO;

  switch (reg & 0x07) {  // Bits 0–2
    case 0b000: state.fan_speed = FanSpeed::AUTO; break;
    case 0b001: state.fan_speed = FanSpeed::MIN; break;
    case 0b010: state.fan_speed = FanSpeed::NIGHT; break;
    case 0b011: state.fan_speed = FanSpeed::MAX; break;
    default:    state.fan_speed = FanSpeed::UNKNOWN; break;
  }

  return state;
}

// Converts Mode to string
const char *to_string(Mode mode) {
  switch (mode) {
    case Mode::OFF:     return "OFF";
    case Mode::HEATING: return "HEATING";
    case Mode::COOLING: return "COOLING";
    case Mode::AUTO:    return "AUTO";
    default:            return "UNKNOWN";
  }
}

// Converts FanSpeed to string
const char *to_string(FanSpeed speed) {
  switch (speed) {
    case FanSpeed::AUTO:    return "AUTO";
    case FanSpeed::MIN:     return "MIN";
    case FanSpeed::NIGHT:   return "NIGHT";
    case FanSpeed::MAX:     return "MAX";
    case FanSpeed::UNKNOWN: return "UNKNOWN";
    default:                return "INVALID";
  }
}

// Logs ParsedState to ESPHome logs
void log_parsed_state(const ParsedState &state) {
  ESP_LOGI(TAG, "Parsed Register 101:");
  ESP_LOGI(TAG, "  Power:     %s", state.on ? "ON" : "OFF");
  ESP_LOGI(TAG, "  Mode:      %s", to_string(state.mode));
  ESP_LOGI(TAG, "  Fan Speed: %s", to_string(state.fan_speed));
}

// ------------------ Bridge Setup ------------------

void OlimpiaBridge::setup() {
  ESP_LOGI(TAG, "Setting up OlimpiaBridge...");

  if (!this->uart_) {
    ESP_LOGE(TAG, "UART not configured.");
    this->mark_failed();
    return;
  }

  // Support single or dual direction pins
  if (this->direction_pin_ != nullptr) {
    this->direction_pin_->setup();
    this->direction_pin_->digital_write(false);  // ⬅️ RX mode
  } else if (this->re_pin_ != nullptr && this->de_pin_ != nullptr) {
    this->re_pin_->setup();
    this->de_pin_->setup();
    this->re_pin_->digital_write(false);  // ⬅️ RX mode
    this->de_pin_->digital_write(false);  // ⬅️ RX mode
  } else {
    ESP_LOGE(TAG, "No direction pin or RE/DE pair configured.");
    this->mark_failed();
    return;
  }

  // --- Conditionally register HA service to write EEPROM configuration registers ---
  if (this->enable_configuration_writes_) {
    this->register_service(&OlimpiaBridge::on_write_register,
                           "olimpia_write_config_register",
                           {"address", "register", "value"});
    ESP_LOGI(TAG, "EEPROM config write service ENABLED via 'enable_configuration_writes'");
  } else {
    ESP_LOGI(TAG, "EEPROM config write service DISABLED (set enable_configuration_writes: true to enable)");
  }

  // --- ALWAYS register a debug HA service to read back any Modbus register ---
  this->register_service(&OlimpiaBridge::on_read_register,
                         "olimpia_read_register",
                         {"address", "register"});
  ESP_LOGI(TAG, "Modbus READ debug service 'olimpia_read_register' registered (address, register)");

  if (this->handler_) {
    this->handler_->set_uart(this->uart_);
    this->handler_->set_direction_pin(this->direction_pin_);
    this->handler_->set_re_pin(this->re_pin_);
    this->handler_->set_de_pin(this->de_pin_);
  }

  for (auto *climate : this->climates_) {
    climate->set_handler(this->handler_);
    climate->setup();
  }
}

// Implementation of the new on_read_register()
void OlimpiaBridge::on_read_register(int address, int reg) {
  ESP_LOGI(TAG, "Service: Reading register %d on address %d", reg, address);

  // Validate inputs
  if (address < 1 || address > 247 || reg < 1 || reg > 9999) {
    ESP_LOGW(TAG, "Invalid address (%d) or register (%d).", address, reg);
    return;
  }
  if (!this->handler_) {
    ESP_LOGW(TAG, "Modbus handler not initialized; read_register skipped");
    return;
  }
  uint16_t value = 0;
  // Attempt the read via our ModbusAsciiHandler
  if (this->handler_->read_register((uint8_t)address, (uint16_t)reg, &value)) {
    ESP_LOGI(TAG, "Read success – addr %d reg %d → 0x%04X (%d)",
             address, reg, value, value);
  } else {
    ESP_LOGW(TAG, "Read failed for addr %d reg %d", address, reg);
  }
}

// Called every update interval to trigger Modbus polling and writing
void OlimpiaBridge::update() {
  for (auto *climate : this->climates_) {
    climate->control_cycle();  // Perform write + read for this unit
    delay(50);                 // Stagger requests between slaves (50ms)
  }
}

// ------------------ Climate ------------------

void OlimpiaBridgeClimate::setup() {
  
  this->current_temperature_ = NAN;
  this->mode = climate::CLIMATE_MODE_OFF;          // External HA climate mode
  this->mode_ = Mode::OFF;                         // Internal control state
  this->on_ = false;                               // Initially off
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->fan_speed_ = FanSpeed::AUTO;
  this->target_temperature = 22.0f;

  // --- Load ambient temp from flash (used only if HA doesn't push it) ---
  this->ambient_temp_pref_ = global_preferences->make_preference<float>(this->address_ * 100 + 3);
  float stored_temp;
  if (this->ambient_temp_pref_.load(&stored_temp)) {
    this->external_ambient_temperature_ = stored_temp;

    // OLD: don't mark as received, used only if HA doesn't push
    this->has_received_external_temp_ = false;

    ESP_LOGI(TAG, "[%s] Loaded fallback ambient temp from flash: %.1f°C", this->get_name().c_str(), stored_temp);

    // NEW: fallback should be treated as valid on cold boot if HA is unavailable
    if (std::isnan(this->external_ambient_temperature_)) {
      ESP_LOGW(TAG, "[%s] Loaded fallback temp was NaN — skipping", this->get_name().c_str());
    } else {
      // NEW: use the fallback value proactively
      this->has_received_external_temp_ = true;
      ESP_LOGI(TAG, "[%s] Using fallback ambient temp as valid: %.1f°C", this->get_name().c_str(), stored_temp);
    }
  } else {
    ESP_LOGW(TAG, "[%s] No saved ambient temp found in flash", this->get_name().c_str());
  }
  // Add your logging statement here for clarity after setup finishes
  ESP_LOGI(TAG, "[%s] Climate setup complete.", this->get_name().c_str());
}

climate::ClimateTraits OlimpiaBridgeClimate::traits() {
  climate::ClimateTraits traits;

  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_AUTO,
  });

  // Expose supported fan modes compatible with Olimpia PRG
  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_HIGH,
    climate::CLIMATE_FAN_QUIET,
  });

  return traits;
}

void OlimpiaBridgeClimate::control(const climate::ClimateCall &call) {
  bool did_write = false;

  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    ESP_LOGI(TAG, "[%s] Target temperature set to %.1f°C", this->get_name().c_str(), this->target_temperature);
    did_write = true;
  }

  if (call.get_mode().has_value()) {
    climate::ClimateMode new_mode = *call.get_mode();
    this->set_mode(new_mode);  // Mode: immediately write + read feedback
    did_write = false;  // Already handled
  }

  if (call.get_fan_mode().has_value()) {
    climate::ClimateFanMode mode = *call.get_fan_mode();

    std::string fan_mode_str;
    switch (mode) {
      case climate::CLIMATE_FAN_AUTO:  fan_mode_str = "auto"; break;
      case climate::CLIMATE_FAN_LOW:   fan_mode_str = "low"; break;
      case climate::CLIMATE_FAN_HIGH:  fan_mode_str = "high"; break;
      case climate::CLIMATE_FAN_QUIET: fan_mode_str = "quiet"; break;
      default:                         fan_mode_str = "auto"; break;
    }

    this->set_fan_mode(fan_mode_str);  // Fan: immediately write + read feedback
    did_write = false;  // Already handled
  }

  // If only temperature changed, we must apply changes manually
  if (did_write) {
    this->write_control_registers();     // Apply new target temperature
    this->control_cycle();               // Trigger readback + publish
  }

  // NOTE: We do not call write_control_registers() or publish_state() here
  // if set_mode() or set_fan_mode() was called, because they already handle it
}

void OlimpiaBridgeClimate::update_climate_action_from_register9() {
  if (this->handler_ == nullptr)
    return;

  // Read real-time status register
  uint16_t reg9 = 0;
  if (!this->handler_->read_register(this->address_, 9, &reg9)) {
    ESP_LOGW(TAG, "[%s] Failed to read register 9", this->get_name().c_str());
    return;
  }
  bool boiler = (reg9 & (1 << 14)) != 0;
  bool chiller = (reg9 & (1 << 15)) != 0;

  ESP_LOGD(TAG, "[%s] Register 9: 0x%04X → boiler: %d, chiller: %d", this->get_name().c_str(), reg9, boiler, chiller);

  // Determine action based on power state (from STBY bit in register 101)
  if (!this->on_) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else {
    if (boiler) {
      this->action = climate::CLIMATE_ACTION_HEATING;
    } else if (chiller) {
      this->action = climate::CLIMATE_ACTION_COOLING;
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
  }
}

void OlimpiaBridgeClimate::set_mode(climate::ClimateMode mode) {
  ESP_LOGD(TAG, "[%s] HVAC mode set to %d", this->get_name().c_str(), mode);

  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      this->on_ = false;
      break;
    case climate::CLIMATE_MODE_COOL:
      this->mode_ = Mode::COOLING;
      this->on_ = true;
      break;
    case climate::CLIMATE_MODE_HEAT:
      this->mode_ = Mode::HEATING;
      this->on_ = true;
      break;
    case climate::CLIMATE_MODE_AUTO:
      this->mode_ = Mode::AUTO;
      this->on_ = true;
      break;
    default:
      ESP_LOGW(TAG, "[%s] Unsupported HVAC mode: %d", this->get_name().c_str(), mode);
      return;
  }

  // 🔁 Immediately write control registers
  this->write_control_registers();

  // 🔁 Immediately read register 9 to determine correct action (idle/heating/cooling/off)
  this->update_climate_action_from_register9();

  // 🔁 Publish updated state to Home Assistant
  this->publish_state();
}

void OlimpiaBridgeClimate::set_fan_mode(const std::string &mode) {
  ESP_LOGD(TAG, "[%s] Fan mode set to '%s'", this->get_name().c_str(), mode.c_str());

  if (mode == "auto") {
    this->fan_speed_ = FanSpeed::AUTO;
  } else if (mode == "low") {
    this->fan_speed_ = FanSpeed::MIN;
  } else if (mode == "quiet") {
    this->fan_speed_ = FanSpeed::NIGHT;
  } else if (mode == "high") {
    this->fan_speed_ = FanSpeed::MAX;
  } else {
    ESP_LOGW(TAG, "[%s] Unknown fan mode '%s'", this->get_name().c_str(), mode.c_str());
    return;
  }

  // 🔁 Immediately write control registers
  this->write_control_registers();

  // 🔁 Refresh register 9 and apply updated climate action
  this->update_climate_action_from_register9();

  // 🔁 Publish updated state to Home Assistant
  this->publish_state();
}

void OlimpiaBridgeClimate::control_cycle() {
  uint32_t now = millis();
  if (!this->initialized_ || now - this->last_update_time_ > 60000) {
    this->write_control_registers();
    this->read_water_temperature();  // purely informative, never affects current_temperature
    this->last_update_time_ = now;
    this->initialized_ = true;

    // --- PATCH 4: Update climate action (idle/heating/cooling) from register 9 ---
    this->update_climate_action_from_register9();

    // --- PATCH 3: Set Home Assistant fan_mode string based on current fan_speed_ ---
    switch (this->fan_speed_) {
      case FanSpeed::AUTO:
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        break;
      case FanSpeed::MIN:
        this->fan_mode = climate::CLIMATE_FAN_LOW;
        break;
      case FanSpeed::NIGHT:
        this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        break;
      case FanSpeed::MAX:
        this->fan_mode = climate::CLIMATE_FAN_HIGH;
        break;
      default:
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        break;
    }

    if (this->fan_mode.has_value()) {
      ESP_LOGD(TAG, "[%s] Fan mode string for HA: %s", this->get_name().c_str(), climate::climate_fan_mode_to_string(*this->fan_mode));
    }

    // FIXED: Only external ambient temperature sets current_temperature_
    if (!std::isnan(this->external_ambient_temperature_)) {
      this->current_temperature_ = this->external_ambient_temperature_;
      ESP_LOGD(TAG, "[%s] Current temperature set from external: %.1f°C", this->get_name().c_str(), this->current_temperature_);
    } else {
      ESP_LOGW(TAG, "[%s] No valid external ambient temperature → current_temperature = NaN", this->get_name().c_str());
      this->current_temperature_ = NAN;
    }

    this->publish_state();  // Publish updated state to Home Assistant
  }
}

float OlimpiaBridgeClimate::get_current_temperature() const {
  ESP_LOGD(TAG, "[%s] get_current_temperature() → %.1f°C", this->get_name().c_str(), this->external_ambient_temperature_);
  return this->current_temperature_;
}

uint16_t OlimpiaBridgeClimate::get_status_register() {
  uint16_t reg = 0;

  // --- Fan Speed Bits (bits 0–2) ---
  if (this->fan_mode.has_value()) {
    switch (*this->fan_mode) {
      case climate::CLIMATE_FAN_AUTO:
        reg |= 0b000;
        break;
      case climate::CLIMATE_FAN_LOW:
        reg |= 0b001;
        break;
      case climate::CLIMATE_FAN_QUIET:
        reg |= 0b010;
        break;
      case climate::CLIMATE_FAN_HIGH:
        reg |= 0b011;
        break;
      default:
        reg |= 0b000;  // fallback to AUTO
        break;
    }
  }

  // --- HVAC Mode Bits (bits 5–6) ---
  switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT:
      reg |= (0b01 << 5);
      break;
    case climate::CLIMATE_MODE_COOL:
      reg |= (0b10 << 5);
      break;
    case climate::CLIMATE_MODE_AUTO:
      reg |= (0b11 << 5);
      break;
    default:
      break;  // Leave bits 5–6 as 00
  }

  // --- Standby Bit (bit 7) ---
  if (this->mode == climate::CLIMATE_MODE_OFF)
    reg |= (1 << 7);  // STBY bit → 1 = OFF

  return reg;
}

void OlimpiaBridgeClimate::write_control_registers() {
  if (this->handler_ == nullptr)
    return;

  // --- Register 101: Status Register ---
  uint16_t status_value = this->get_status_register();
  this->handler_->write_register(this->address_, 101, status_value);
  ESP_LOGD(TAG, "[%s] Wrote status register (101): 0x%04X", this->get_name().c_str(), status_value);

  delay(20);  // Delay before writing temperature

  // --- Register 102: Target Temperature (°C × 10) ---
  uint16_t target_temp = static_cast<uint16_t>(this->target_temperature * 10);
  this->handler_->write_register(this->address_, 102, target_temp);
  ESP_LOGD(TAG, "[%s] Wrote target temperature (102): %.1f°C", this->get_name().c_str(), this->target_temperature);

  delay(20);  // Delay before writing status

  // --- Register 103: Ambient Temperature from HA (or fallback) ---
  if (!std::isnan(this->external_ambient_temperature_)) {
    uint16_t ambient_temp = static_cast<uint16_t>(this->external_ambient_temperature_ * 10);

    ESP_LOGI(TAG, "[%s] Preparing to write ambient temperature (103): %.1f°C → 0x%04X",
             this->get_name().c_str(), this->external_ambient_temperature_, ambient_temp);

    this->handler_->write_register(this->address_, 103, ambient_temp);
    ESP_LOGD(TAG, "[%s] Wrote ambient temperature (103)", this->get_name().c_str());

    delay(20);  // Extra delay before writing other registers

    // --- Save once per day to EEPROM for reboot fallback ---
    uint32_t now = millis();
    if (this->has_received_external_temp_ && (now - this->last_persist_time_ > 86400000UL)) {
      this->ambient_temp_pref_.save(&this->external_ambient_temperature_);
      this->last_persist_time_ = now;
      ESP_LOGI(TAG, "[%s] Saved ambient temp to flash: %.1f°C", this->get_name().c_str(), this->external_ambient_temperature_);
    }
  } else {
    ESP_LOGW(TAG, "[%s] Ambient temperature unavailable, skipping register 103", this->get_name().c_str());
  }
}

// Read water temperature from register 1 and publish it
void OlimpiaBridgeClimate::read_water_temperature() {
  if (this->handler_ == nullptr || this->water_temp_sensor_ == nullptr)
    return;

  uint16_t value = 0;
  if (this->handler_->read_register(this->address_, 1, &value)) { 
    float celsius = value / 10.0f;
    this->current_temperature_ = celsius;
    this->water_temp_sensor_->publish_state(celsius);
  }
}

void OlimpiaBridge::increment_modbus_request(bool success) {
  this->modbus_total_requests_++;
  if (!success)
    this->modbus_failed_requests_++;

  if (this->modbus_error_ratio_sensor_ != nullptr && this->modbus_total_requests_ > 0) {
    float ratio = 100.0f * static_cast<float>(this->modbus_failed_requests_) /
                  static_cast<float>(this->modbus_total_requests_);
    this->modbus_error_ratio_sensor_->publish_state(ratio);
  }
}

// Set external ambient temperature from HA with validation
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  if (std::isnan(temp) || temp < 0.0f || temp > 50.0f) {
    ESP_LOGW(TAG, "[%s] Ignoring invalid external temperature: %.1f°C", this->get_name().c_str(), temp);
    return;
  }

  this->external_ambient_temperature_ = temp;
  this->has_received_external_temp_ = true;
  ESP_LOGI(TAG, "[%s] Received new external ambient temperature: %.1f°C", this->get_name().c_str(), temp);
}

void OlimpiaBridge::on_write_register(int address, int reg, int value) {
  ESP_LOGI(TAG, "Service: Writing value %d to register %d on address %d", value, reg, address);

  // Validate Modbus address and register range
  if (address < 1 || address > 247 || reg < 200 || reg > 255) {
    ESP_LOGW(TAG, "Write to register %d blocked — only registers 200–255 are permitted for configuration writes", reg);
    return;
  }

  // Block write if configuration service is not enabled
  if (!this->enable_configuration_writes_) {
    ESP_LOGW(TAG, "Write to register %d denied — configuration writes are disabled", reg);
    return;
  }

  // Attempt to write if handler is ready
  if (this->handler_ != nullptr) {
    this->handler_->write_register(static_cast<uint8_t>(address),
                                   static_cast<uint16_t>(reg),
                                   static_cast<uint16_t>(value));
    ESP_LOGD(TAG, "Register write complete");
  } else {
    ESP_LOGW(TAG, "Modbus handler not initialized; write_register skipped");
  }
}

}  // namespace olimpia_bridge
}  // namespace esphome
