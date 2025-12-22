#include "esphome/core/log.h"
#include "olimpia_bridge_climate.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "climate";

namespace {

struct ModeInfo {
  Mode value;
  const char* name;
  climate::ClimateMode climate_mode;
};

struct FanSpeedInfo {
  FanSpeed value;
  const char* name;
  climate::ClimateFanMode climate_fan_mode;
};

static constexpr ModeInfo MODE_INFO[] = {
    {Mode::AUTO, "AUTO", climate::CLIMATE_MODE_AUTO},
    {Mode::HEATING, "HEAT", climate::CLIMATE_MODE_HEAT},
    {Mode::COOLING, "COOL", climate::CLIMATE_MODE_COOL},
};

static constexpr FanSpeedInfo FAN_SPEED_INFO[] = {
    {FanSpeed::AUTO, "AUTO", climate::CLIMATE_FAN_AUTO},
    {FanSpeed::MIN, "LOW", climate::CLIMATE_FAN_LOW},
    {FanSpeed::NIGHT, "QUIET", climate::CLIMATE_FAN_QUIET},
    {FanSpeed::MAX, "HIGH", climate::CLIMATE_FAN_HIGH},
};

// Helper to find ModeInfo by Mode enum
const ModeInfo& get_mode_info(Mode mode) {
  for (const auto& info : MODE_INFO) {
    if (info.value == mode) return info;
  }
  return MODE_INFO[0]; // Default to AUTO
}

// Helper to find FanSpeedInfo by FanSpeed enum
const FanSpeedInfo& get_fan_speed_info(FanSpeed fan_speed) {
  for (const auto& info : FAN_SPEED_INFO) {
    if (info.value == fan_speed) return info;
  }
  return FAN_SPEED_INFO[0]; // Default to AUTO
}

} // anonymous namespace

// --- Helper functions for logging ---
static const char *mode_to_string(Mode mode) {
  return get_mode_info(mode).name;
}

static const char *fan_speed_to_string(FanSpeed fan) {
  return get_fan_speed_info(fan).name;
}

static std::string presets_to_uppercase(const std::string &str) {
  std::string out = str;
  for (auto &c : out) c = toupper(c);
  return out;
}

// --- Error ratio publish helpers ---
void OlimpiaBridgeClimate::publish_device_error_ratio_if_enabled() {
  if (!this->device_error_ratio_sensor_ || this->device_total_requests_ == 0) return;
  int ratio = static_cast<int>((100.0f * this->device_failed_requests_) / this->device_total_requests_ + 0.5f);
  if (ratio == this->last_published_device_error_ratio_) return;
  this->device_error_ratio_sensor_->publish_state(ratio);
  this->last_published_device_error_ratio_ = ratio;
}

// --- Centralized state mapping and publishing ---
void OlimpiaBridgeClimate::sync_and_publish() {
  if (this->on_) {
    this->mode = get_mode_info(this->mode_).climate_mode;
    this->fan_mode = get_fan_speed_info(this->fan_speed_).climate_fan_mode;
  } else {
    this->mode = climate::CLIMATE_MODE_OFF;
    this->fan_mode = get_fan_speed_info(this->fan_speed_).climate_fan_mode; // Fan mode is still relevant when off
  }
  
  this->current_temperature = this->external_ambient_temperature_;

  if (this->presets_enabled_) {
    if (this->custom_preset_ == "Auto" || this->custom_preset_ == "Manual") {
      this->set_custom_preset_(this->custom_preset_.c_str());
    }
  }

  this->publish_state();
}

// --- Compose Register 101: Bit-mapped control flags ---
uint16_t OlimpiaBridgeClimate::build_command_register(bool on, Mode mode, FanSpeed fan_speed) {
  uint16_t reg = 0;

  // Bits 0–2: PRG fan speed
  // 000 = Auto, 001 = Min, 010 = Night, 011 = Max
  reg |= static_cast<uint8_t>(fan_speed) & 0x07;

  // Bit 7: STBY (working condition)
  // 0 = Activated, 1 = Standby (OFF)
  if (!on)
    reg |= (1 << 7);

  // Bit 12: CP presence contact
  reg &= ~(1 << 12);  // Ensure CP is always 0

  // Bits 13–14: EI mode of functioning
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
  ESP_LOGCONFIG(TAG, "[%s] Setting up Olimpia Bridge Climate...", this->get_name().c_str());
  this->system_boot_time_ms_ = millis();

  // Initialize preferences storage
  this->pref_ = global_preferences->make_preference<SavedState>(this->get_object_id_hash());
  if (this->pref_.load(&this->state_)) {
    ESP_LOGI(TAG, "[%s] Restored state from flash", this->get_name().c_str());
    this->on_ = this->state_.on;
    this->mode_ = this->state_.mode;
    this->fan_speed_ = this->state_.fan_speed;
    this->target_temperature = this->state_.target_temperature;
    this->action = this->state_.last_action;
    this->custom_preset_ = this->state_.custom_preset;
    this->external_ambient_temperature_ = this->state_.last_ambient_temperature;
    this->current_temperature = this->state_.last_ambient_temperature;
    this->using_fallback_external_temp_ = true;
  } else {
    ESP_LOGI(TAG, "[%s] No saved state found, using defaults.", this->get_name().c_str());
    this->target_temperature = 22.0f; // Restore default target temperature
  }

  // Read water temp from device
  this->read_water_temperature();

  // Start boot recovery or sync state
  this->restore_or_refresh_state();

  // Randomize per-device periodic poll intervals
  this->next_status_poll_ms_ = millis() + random(0, INITIAL_POLL_JITTER_MS);
}

// --- Traits ---
climate::ClimateTraits OlimpiaBridgeClimate::traits() {
  climate::ClimateTraits traits;
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);

  // Conditionally expose AUTO mode based on config
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  if (!this->disable_mode_auto_) {
    traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);
  }

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  if (!this->disable_fan_quiet_) {
    traits.add_supported_fan_mode(climate::CLIMATE_FAN_QUIET);
  }
  traits.set_visual_current_temperature_step(0.1);
  traits.set_visual_target_temperature_step(this->target_temperature_step_);

  // Ensure min and max temperature traits are optional
  if (!std::isnan(this->min_temperature_)) {
    traits.set_visual_min_temperature(this->min_temperature_);
  }
  if (!std::isnan(this->max_temperature_)) {
    traits.set_visual_max_temperature(this->max_temperature_);
  }

  // Update traits to conditionally expose presets
  if (this->presets_enabled_) {
    traits.set_supported_custom_presets({"Auto", "Manual"});
  }

  return traits;
}

// --- Home Assistant control callback ---
void OlimpiaBridgeClimate::control(const climate::ClimateCall &call) {
  if (this->component_state_ != ComponentState::RUNNING) {
    ESP_LOGW(TAG, "[%s] Control is blocked during boot/recovery.", this->get_name().c_str());
    return;
  }

  bool state_changed = false;

  // Handle target temperature change
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    ESP_LOGI(TAG, "[%s] Target temperature set to %.1f°C", this->get_name().c_str(), this->target_temperature);
    state_changed = true;
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_OFF:
        this->on_ = false;
        this->mode_ = Mode::AUTO;  // OFF = on_=false + AUTO
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

  // Handle fan mode change
  if (call.get_fan_mode().has_value()) {
    auto fan = *call.get_fan_mode();
    if (fan == climate::CLIMATE_FAN_AUTO) this->fan_speed_ = FanSpeed::AUTO;
    else if (fan == climate::CLIMATE_FAN_LOW) this->fan_speed_ = FanSpeed::MIN;
    else if (fan == climate::CLIMATE_FAN_QUIET) this->fan_speed_ = FanSpeed::NIGHT;
    else if (fan == climate::CLIMATE_FAN_HIGH) this->fan_speed_ = FanSpeed::MAX;
    state_changed = true;
  }

  // Handle custom preset change
  if (this->presets_enabled_ && call.get_custom_preset() != nullptr) {
    this->custom_preset_ = call.get_custom_preset();
    state_changed = true;
  }

  if (state_changed) {
    this->save_state_to_flash();
    this->write_control_registers_cycle([this]() {
      this->update_climate_action_from_valve_status();
    });
  }
}

// --- Save State to Flash ---
void OlimpiaBridgeClimate::save_state_to_flash() {
  this->state_.on = this->on_;
  this->state_.mode = this->mode_;
  this->state_.fan_speed = this->fan_speed_;
  this->state_.target_temperature = this->target_temperature;
  this->state_.last_action = this->action;
  strncpy(this->state_.custom_preset, this->custom_preset_.c_str(), sizeof(this->state_.custom_preset) - 1);
  this->state_.last_ambient_temperature = this->external_ambient_temperature_;

  if (this->pref_.save(&this->state_)) {
    ESP_LOGD(TAG, "[%s] Saved state to flash", this->get_name().c_str());
  } else {
    ESP_LOGW(TAG, "[%s] Failed to save state to flash", this->get_name().c_str());
  }
}

// --- Write Control Registers Cycle ---
void OlimpiaBridgeClimate::write_control_registers_cycle(std::function<void()> callback) {
  if (this->handler_ == nullptr) return;

  uint16_t reg101 = this->get_status_register();
  uint16_t reg102 = static_cast<uint16_t>(this->target_temperature * 10);
  uint16_t reg103 = std::isnan(this->external_ambient_temperature_) ? 0 : static_cast<uint16_t>(this->external_ambient_temperature_ * 10);

  ESP_LOGI(TAG, "[%s] Writing control → Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C | Ambient: %.1f°C",
           this->get_name().c_str(),
           this->on_ ? "ON" : "OFF",
           mode_to_string(this->mode_),
           fan_speed_to_string(this->fan_speed_),
           presets_to_uppercase(this->custom_preset_).c_str(),
           this->target_temperature,
           this->external_ambient_temperature_);

  this->handler_->write_register(this->address_, 101, reg101, [this, reg102, reg103, callback](bool ok1, const std::vector<uint16_t> &) {
    this->device_total_requests_++;
    // Coalesce and publish ratio after activity
    this->publish_device_error_ratio_if_enabled();
    if (!ok1) {
      this->device_failed_requests_++;
      this->publish_device_error_ratio_if_enabled();
      ESP_LOGW(TAG, "[%s] Failed to write register 101", this->get_name().c_str());
      return;
    }

    this->handler_->write_register(this->address_, 102, reg102, [this, reg103, callback](bool ok2, const std::vector<uint16_t> &) {
      this->device_total_requests_++;
      this->publish_device_error_ratio_if_enabled();
      if (!ok2) {
        this->device_failed_requests_++;
        this->publish_device_error_ratio_if_enabled();
        ESP_LOGW(TAG, "[%s] Failed to write register 102", this->get_name().c_str());
        return;
      }

      this->handler_->write_register(this->address_, 103, reg103, [this, callback](bool ok3, const std::vector<uint16_t> &) {
        this->device_total_requests_++;
        this->publish_device_error_ratio_if_enabled();
        if (!ok3) {
          this->device_failed_requests_++;
          this->publish_device_error_ratio_if_enabled();
          ESP_LOGW(TAG, "[%s] Failed to write register 103", this->get_name().c_str());
          return;
        }

        // Allow device time to process before action check
        if (callback) {
          this->set_timeout("valve_status_check", VALVE_STATUS_CHECK_DELAY_MS, [callback]() {
            callback();
          });
        }
      });
    });
  });
}

// --- Parse and Apply Register 101 State ---
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {
  // Copy state from parsed register 101
  this->on_ = parsed.on;
  this->mode_ = parsed.mode;
  this->fan_speed_ = parsed.fan_speed;

  // Always restore the preset from flash, since the device does not store it
  this->custom_preset_ = this->state_.custom_preset;

  ESP_LOGD(TAG, "[%s] Updated state from reg101: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C | Ambient: %.1f°C",
           this->get_name().c_str(),
           this->on_ ? "ON" : "OFF",
           mode_to_string(this->mode_),
           fan_speed_to_string(this->fan_speed_),
           presets_to_uppercase(this->custom_preset_).c_str(),
           this->target_temperature,
           this->current_temperature);
  
  // Update action and publish state to HA
  this->update_climate_action_from_valve_status();
}

// --- Compose Register 101 from State ---
uint16_t OlimpiaBridgeClimate::get_status_register() {
  return this->build_command_register(this->on_, this->mode_, this->fan_speed_);
}

// --- External Ambient Temperature from HA ---
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  if (std::isnan(temp)) return;

  const uint32_t now = millis();
  bool first_time = !this->has_received_external_temp_;
  bool refresh_flash = (now - this->last_external_temp_flash_write_ > EXTERNAL_TEMP_FLASH_WRITE_INTERVAL_MS);
  bool temp_changed = std::abs(temp - this->external_ambient_temperature_) > 0.05f;

  // If EMA is disabled, bypass all logic and just update the temperature
  if (!this->use_ema_) {
    if (temp_changed) {
      this->external_ambient_temperature_ = temp;
      this->current_temperature = temp;
      this->publish_state();
      ESP_LOGD(TAG, "[%s] EMA disabled. Ambient set to: %.1f°C", this->get_name().c_str(), temp);
    }
    if (first_time || refresh_flash) {
      this->save_state_to_flash();
      this->last_external_temp_flash_write_ = now;
    }
    return;
  }

  // Handle fallback and first-HA reception
  if (!this->has_received_external_temp_ && this->using_fallback_external_temp_) {
    ESP_LOGI(TAG, "[%s] Restoring last known ambient from FLASH: %.1f°C", this->get_name().c_str(), temp);
    this->first_ha_ambient_received_ = false;  // Ensure bypass still happens on next HA update
    this->smoothed_ambient_ = NAN;  // Defensive reset for EMA after reboots
  } else if (!this->first_ha_ambient_received_) {
    ESP_LOGI(TAG, "[%s] Fresh ambient received from HA: %.1f°C, enabling EMA!", this->get_name().c_str(), temp);
    this->first_ha_ambient_received_ = true;
    this->smoothed_ambient_ = NAN;  // Reset EMA

    // Immediately apply the first HA value
    this->external_ambient_temperature_ = temp;
    this->current_temperature = temp;
    this->has_received_external_temp_ = true;
    this->external_temp_received_from_ha_ = true;
    this->last_external_temp_update_ = now;
    this->publish_state();
    return;
  } else {
    // Smart EMA Reset if inactive too long
    if (now - this->last_external_temp_update_ > EMA_INACTIVITY_RESET_MS) {
      ESP_LOGI(TAG, "[%s] EMA reset due to inactivity. Accepting new ambient: %.1f°C", this->get_name().c_str(), temp);
      this->external_ambient_temperature_ = temp;
      this->current_temperature = temp;
      this->smoothed_ambient_ = temp;
      this->has_received_external_temp_ = true;
      this->external_temp_received_from_ha_ = true;
      this->last_external_temp_update_ = now;
      this->publish_state();
      return;
    }

    // Exponential Moving Average smoothing logic with trend-based early confirmation logic
    float prev = this->smoothed_ambient_;
    float ema = std::isnan(prev) ? temp : this->ambient_ema_alpha_ * temp + (1.0f - this->ambient_ema_alpha_) * prev;

    float rounded = std::round(ema * 10.0f) / 10.0f;
    float trend = ema - prev;

    this->smoothed_ambient_ = ema;

    if (rounded != this->external_ambient_temperature_) {
      bool should_confirm = false;
      if (rounded > this->external_ambient_temperature_) {
        should_confirm = trend > 0 && ema >= (rounded - 0.02f);
      } else if (rounded < this->external_ambient_temperature_) {
        should_confirm = trend < 0 && ema <= (rounded + 0.02f);
      }

      const char *trend_str = (trend > 0) ? "↑" : (trend < 0) ? "↓" : "→";

      if (!should_confirm) {
        ESP_LOGI(TAG, "[%s] EMA rejected %.1f°C from HA → EMA %.2f°C (trend %s, held %.1f°C)",
                 this->get_name().c_str(), temp, ema, trend_str, this->external_ambient_temperature_);
        return;
      }

      temp = rounded;
      ESP_LOGI(TAG, "[%s] EMA accepted %.1f°C from HA → EMA %.2f°C (trend %s)",
               this->get_name().c_str(), temp, ema, trend_str);
    } else {
      ESP_LOGI(TAG, "[%s] EMA stable: %.1f°C (EMA %.2f°C)", this->get_name().c_str(), temp, ema);
    }
  }
  // Update memory state
  this->external_ambient_temperature_ = temp;
  this->current_temperature = temp;
  this->has_received_external_temp_ = true;
  this->external_temp_received_from_ha_ = true;
  this->last_external_temp_update_ = now;
  this->publish_state();
}

// --- Read Water Temperature ---
void OlimpiaBridgeClimate::read_water_temperature() {
  if (this->water_temp_sensor_ == nullptr) {
    return;
  }
  if (this->handler_ == nullptr) {
    ESP_LOGW(TAG, "[%s] Modbus handler not available, cannot read water temperature.", this->get_name().c_str());
    return;
  }
  this->handler_->read_register(this->address_, 1, 1, [this](bool success, const std::vector<uint16_t> &data) {
    this->device_total_requests_++;
    this->publish_device_error_ratio_if_enabled();
    if (!success || data.empty()) {
      this->device_failed_requests_++;
      this->publish_device_error_ratio_if_enabled();
      ESP_LOGW(TAG, "[%s] Failed to read register 1 (water temperature)", this->get_name().c_str());
      return;
    }
    float temp = data[0] / 10.0f;
    ESP_LOGD(TAG, "[%s] Read water temperature: %.1f°C", this->get_name().c_str(), temp);
    this->water_temp_sensor_->publish_state(temp);
  });
}

// --- Restore Saved State from Flash ---
void OlimpiaBridgeClimate::apply_last_known_state() {
  this->on_ = this->state_.on;
  this->mode_ = this->state_.mode;
  this->fan_speed_ = this->state_.fan_speed;
  this->target_temperature = this->state_.target_temperature;
  this->custom_preset_ = this->state_.custom_preset;
  ESP_LOGI(TAG, "[%s] Applying last known state from flash", this->get_name().c_str());
}

// --- Boot/Recovery State Restoration ---
void OlimpiaBridgeClimate::restore_or_refresh_state() {
  if (this->handler_ == nullptr) return;

  if (this->component_state_ == ComponentState::RECOVERING) {
    ESP_LOGD(TAG, "[%s] Recovery already in progress, skipping.", this->get_name().c_str());
    return;
  }

  bool is_boot_cycle = (this->component_state_ == ComponentState::BOOTING);

  if (is_boot_cycle) {
    ESP_LOGI(TAG, "[%s] Boot state recovery: reading 101 + 102...", this->get_name().c_str());
    this->component_state_ = ComponentState::RECOVERING;
  } else {
    ESP_LOGD(TAG, "[%s] Refreshing state from device...", this->get_name().c_str());
  }

  this->handler_->read_register(this->address_, 101, 1,
    [this, is_boot_cycle](bool ok101, const std::vector<uint16_t> &data101) {
      this->device_total_requests_++;
      this->publish_device_error_ratio_if_enabled();
      if (!ok101 || data101.empty()) {
        this->device_failed_requests_++;
        this->publish_device_error_ratio_if_enabled();
        ESP_LOGW(TAG, "[%s] Failed to read register 101", this->get_name().c_str());
        if (is_boot_cycle) this->component_state_ = ComponentState::BOOTING; // Allow retry
        return;
      }

      uint16_t reg101 = data101[0];
      ParsedState parsed = parse_command_register(reg101);

      ESP_LOGD(TAG, "[%s] Read 101: 0x%04X → ON=%d MODE=%d FAN=%d", this->get_name().c_str(),
               reg101, parsed.on, parsed.mode, static_cast<int>(parsed.fan_speed));

      this->handler_->read_register(this->address_, 102, 1,
        [this, parsed, is_boot_cycle](bool ok102, const std::vector<uint16_t> &data102) {
          this->device_total_requests_++;
          this->publish_device_error_ratio_if_enabled();
          if (!ok102 || data102.empty()) {
            this->device_failed_requests_++;
            this->publish_device_error_ratio_if_enabled();
            ESP_LOGW(TAG, "[%s] Failed to read register 102", this->get_name().c_str());
            if (is_boot_cycle) this->component_state_ = ComponentState::BOOTING; // Allow retry
            return;
          }

          float target = data102[0] * 0.1f;
          ESP_LOGD(TAG, "[%s] Read 102 → target temperature: %.1f°C", this->get_name().c_str(), target);

          // --- Power-Loss Recovery ---
          if (is_boot_cycle && parsed.mode == Mode::AUTO && std::abs(target - 22.0f) < 0.2f) {
            ESP_LOGW(TAG, "[%s] Detected fallback state (AUTO + 22.0°C), restoring from flash", this->get_name().c_str());
            this->apply_last_known_state();
            this->write_control_registers_cycle([this]() {
              this->restore_or_refresh_state(); // Re-sync after write
              this->component_state_ = ComponentState::RUNNING;
              ESP_LOGI(TAG, "[%s] Boot recovery fallback applied. Component is RUNNING.", this->get_name().c_str());
            });
            return;
          }

          // Set recovered target temperature
          this->target_temperature = target;
          this->update_state_from_parsed(parsed);  // Update internal + publish to HA

          ESP_LOGD(TAG, "[%s] Updated state → ON=%d MODE=%d FAN=%d target=%.1f°C",
                   this->get_name().c_str(), this->on_, static_cast<int>(this->mode_),
                   static_cast<int>(this->fan_speed_), this->target_temperature);

          if (is_boot_cycle) {
            this->component_state_ = ComponentState::RUNNING;
            ESP_LOGI(TAG, "[%s] Boot register read complete. Component is RUNNING.", this->get_name().c_str());
          }
        });
    });
}

// --- Valve Status → Climate Action Mapping ---
void OlimpiaBridgeClimate::update_climate_action_from_valve_status() {
  if (this->handler_ == nullptr) return;

  // Publish the primary state change immediately for UI responsiveness
  this->sync_and_publish();

  this->handler_->read_register(this->address_, 9, 1,
    [this](bool success, const std::vector<uint16_t> &data) {
      this->device_total_requests_++;
      if (!success || data.empty()) {
        this->device_failed_requests_++;
        this->publish_device_error_ratio_if_enabled();
        ESP_LOGW(TAG, "[%s] Failed to read register 9 (valve status)", this->get_name().c_str());
        return;
      }
      this->publish_device_error_ratio_if_enabled();

      const uint16_t reg9 = data[0];
      const uint8_t high_byte = (reg9 >> 8) & 0xFF;

      // Bit meaning based on reverse-engineered protocol
      bool ev1     = (high_byte & 0b01000000) != 0;  // Bit 6
      bool boiler  = (high_byte & 0b00100000) != 0;  // Bit 5
      bool chiller = (high_byte & 0b00010000) != 0;  // Bit 4
      auto new_action = this->action;  // Start with current action as default
      if (!this->on_) {
        new_action = climate::CLIMATE_ACTION_OFF;
      } else if (ev1) {
        // Determine based on current mode
        if (this->mode_ == Mode::COOLING) {
          new_action = climate::CLIMATE_ACTION_COOLING;
        } else if (this->mode_ == Mode::HEATING) {
          new_action = climate::CLIMATE_ACTION_HEATING;
        } else if (this->mode_ == Mode::AUTO) {
          // AUTO requires additional inference
          if (boiler && !chiller)
            new_action = climate::CLIMATE_ACTION_HEATING;
          else if (chiller && !boiler)
            new_action = climate::CLIMATE_ACTION_COOLING;
          else
            new_action = climate::CLIMATE_ACTION_IDLE;  // Both off or invalid
        } else {
          new_action = climate::CLIMATE_ACTION_IDLE;
        }
      } else {
        new_action = climate::CLIMATE_ACTION_IDLE;
      }

      if (this->action != new_action) {
        this->action = new_action;
        ESP_LOGD(TAG, "[%s] Updated action from valve status (reg 9 = 0x%04X): ev1=%d boiler=%d chiller=%d → %s",
                this->get_name().c_str(), reg9, ev1, boiler, chiller,
                climate::climate_action_to_string(new_action));
      } else {
        ESP_LOGD(TAG, "[%s] Valve status unchanged (reg 9 = 0x%04X): action=%s",
                this->get_name().c_str(), reg9,
                climate::climate_action_to_string(this->action));
      }
      // Always publish state, even if action is unchanged, to sync other properties
      this->sync_and_publish();
    });
}

// --- Main Loop: Schedule Updates ---
void OlimpiaBridgeClimate::loop() {
  const uint32_t now = millis();

  // Handle initial boot/recovery retries if stuck
  if (this->component_state_ != ComponentState::RUNNING && now >= this->next_recovery_attempt_ms_) {
    ESP_LOGW(TAG, "[%s] Component is not running, attempting recovery...", this->get_name().c_str());
    this->restore_or_refresh_state();
    this->next_recovery_attempt_ms_ = now + RECOVERY_RETRY_INTERVAL_MS;  // Retry every 15 seconds
  }

  // Unified 60s periodic sync cycle, only when running
  if (this->component_state_ == ComponentState::RUNNING && now >= this->next_status_poll_ms_) {
    this->next_status_poll_ms_ = now + PERIODIC_SYNC_INTERVAL_MS + random(0, PERIODIC_SYNC_JITTER_MS);  // 60s + jitter
    this->periodic_sync();
  }

  // Per-device error ratio sensor update
  //if (this->device_error_ratio_sensor_ && this->device_total_requests_ > 0) {
  //  float error_ratio = (100.0f * this->device_failed_requests_) / this->device_total_requests_;
  //  this->device_error_ratio_sensor_->publish_state(static_cast<int>(error_ratio + 0.5f));
  //}
}

// --- Periodic Sync ---
void OlimpiaBridgeClimate::periodic_sync() {
  ESP_LOGD(TAG, "[%s] Starting periodic sync...", this->get_name().c_str());

  // Periodically re-write the control registers to act as a keep-alive
  // and ensure the HA state is enforced on the device.
  this->write_control_registers_cycle([this]() {
    // After re-asserting the state, check the valve status to update the action.
    this->update_climate_action_from_valve_status();
  });

  // Also, refresh the water temperature reading periodically.
  this->read_water_temperature();
}

}  // namespace olimpia_bridge
}  // namespace esphome
