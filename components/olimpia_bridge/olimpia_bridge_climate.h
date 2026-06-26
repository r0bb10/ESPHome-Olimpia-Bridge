#pragma once

#include <cmath>  // Required for NAN
#include "esphome/core/preferences.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "modbus_ascii_handler.h"

namespace esphome {
namespace olimpia_bridge {

// Timing constants (in milliseconds)
static constexpr uint32_t VALVE_STATUS_CHECK_DELAY_MS = 100;
static constexpr uint32_t RECOVERY_RETRY_INTERVAL_MS = 15000;
static constexpr uint32_t PERIODIC_SYNC_INTERVAL_MS = 60000;
static constexpr uint32_t PERIODIC_SYNC_JITTER_MS = 5000;
static constexpr uint32_t INITIAL_POLL_JITTER_MS = 30000;
static constexpr uint32_t EXTERNAL_TEMP_FLASH_WRITE_INTERVAL_MS = 3600000;  // 1 hour
static constexpr uint32_t EMA_INACTIVITY_RESET_MS = 900000;                  // 15 minutes

// --- Operating modes (EI field in register 101) ---
enum class Mode : uint8_t {
  AUTO     = 0b00,
  HEATING  = 0b01,
  COOLING  = 0b10,
};

// --- Fan speed levels (PRG bits 0–2) ---
enum class FanSpeed : uint8_t {
  AUTO    = 0b000,
  MIN     = 0b001,
  NIGHT   = 0b010,
  MAX     = 0b011,
};

// --- Component State ---
enum class ComponentState : uint8_t {
  BOOTING,
  RECOVERING,
  RUNNING,
};

// --- Parsed register 101 state ---
struct ParsedState {
  bool on = false;
  bool cp = false;
  FanSpeed fan_speed = FanSpeed::AUTO;
  Mode mode = Mode::AUTO;
};

// --- Persisted state structure ---
struct SavedState {
  bool on;
  Mode mode;
  FanSpeed fan_speed;
  float target_temperature;
  climate::ClimateAction last_action;
  char custom_preset[16];  // Fixed-size array for custom preset
  float last_ambient_temperature; // Add ambient temp to persisted state
  bool swing_on;  // Added at the end to preserve existing flash layout
};

// --- Utility to decode register 101 ---
inline ParsedState parse_command_register(uint16_t reg) {
  ParsedState st;

  st.on = (reg & (1 << 7)) == 0;            // Bit 7 = STBY (0 = ON)
  st.cp = (reg & (1 << 12)) != 0;           // Bit 12 = CP
  uint8_t mode = (reg >> 13) & 0x03;        // Bits 13–14 = Mode
  uint8_t fan = reg & 0x07;                 // Bits 0–2 = Fan speed

  switch (mode) {
    case 0b00: st.mode = Mode::AUTO; break;
    case 0b01: st.mode = Mode::HEATING; break;
    case 0b10: st.mode = Mode::COOLING; break;
    default:   st.mode = Mode::AUTO; break;
  }

  switch (fan) {
    case 0x00: st.fan_speed = FanSpeed::AUTO; break;
    case 0x01: st.fan_speed = FanSpeed::MIN; break;
    case 0x02: st.fan_speed = FanSpeed::NIGHT; break;
    case 0x03: st.fan_speed = FanSpeed::MAX; break;
    default:   st.fan_speed = FanSpeed::AUTO; break;
  }

  return st;
}

// --- OlimpiaBridgeClimate class ---
class OlimpiaBridgeClimate : public climate::Climate, public Component {
 public:
  // Core component methods
  void setup() override;
  void loop() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // Configuration
  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }
  void set_error_ratio_sensor(sensor::Sensor *sensor) { this->device_error_ratio_sensor_ = sensor; }
  void set_external_ambient_temperature(float temp);
  void set_ambient_ema_alpha(float alpha) { this->ambient_ema_alpha_ = alpha; }
  void set_use_ema(bool use_ema) { this->use_ema_ = use_ema; }
  void set_min_temperature(float min_temp) { this->min_temperature_ = min_temp; }
  void set_max_temperature(float max_temp) { this->max_temperature_ = max_temp; }
  void set_target_temperature_step(float step) { this->target_temperature_step_ = step; }
  void set_disable_mode_auto(bool disable) { this->disable_mode_auto_ = disable; }
  void set_disable_fan_quiet(bool disable) { this->disable_fan_quiet_ = disable; }
  void set_presets_enabled(bool enabled) { this->presets_enabled_ = enabled; }
  void set_swing_enabled(bool enabled) { this->swing_enabled_ = enabled; }

 protected:
  // State sync and publish
  void sync_and_publish();

  // State control and polling
  void periodic_sync();
  void apply_last_known_state();
  void restore_or_refresh_state();
  void update_climate_action_from_valve_status();
  void read_water_temperature();
  void read_swing_state();
  void write_swing_if_needed(std::function<void()> callback = nullptr);

  // Register management
  uint16_t build_command_register(bool on, Mode mode, FanSpeed fan_speed);
  void write_control_registers_cycle(std::function<void()> callback = nullptr);

  // Internal state updates
  void update_state_from_parsed(const ParsedState &parsed);
  uint16_t get_status_register();
  void save_state_to_flash();

  // Modbus configuration
  uint8_t address_;
  ModbusAsciiHandler *handler_{nullptr};

  // Sensors
  sensor::Sensor *water_temp_sensor_{nullptr};
  sensor::Sensor *device_error_ratio_sensor_{nullptr};

  // Climate state
  float current_temperature_{NAN};
  float external_ambient_temperature_{NAN};
  float smoothed_ambient_{NAN};
  float ambient_ema_alpha_{0.2f};
  bool use_ema_{true};
  bool first_ha_ambient_received_{false};
  uint32_t last_external_temp_update_{0};

  // Device power and control state
  bool on_{false};
  ComponentState component_state_{ComponentState::BOOTING};
  Mode mode_{Mode::AUTO};
  FanSpeed fan_speed_{FanSpeed::AUTO};

  // Persistence
  ESPPreferenceObject pref_;
  SavedState state_{};

  // Custom virtual presets
  std::string custom_preset_{"Auto"};
  bool presets_enabled_ = true;

  // Swing control
  bool swing_enabled_ = false;
  bool swing_on_ = false;
  bool swing_read_once_ = false;

  // Disable AUTO mode and QUIET fan in HA
  bool disable_mode_auto_ = false;
  bool disable_fan_quiet_ = false;

  // External Temperature Management
  bool using_fallback_external_temp_{false};
  bool has_received_external_temp_{false};
  bool external_temp_received_from_ha_{false};
  bool reg103_read_from_device_{false};  // verify
  uint32_t last_external_temp_flash_write_{0};

  // Timing
  uint32_t system_boot_time_ms_{0};
  uint32_t next_status_poll_ms_{0};
  uint32_t next_recovery_attempt_ms_{0};

  // Per-device Modbus request tracking
  uint32_t device_total_requests_{0};
  uint32_t device_failed_requests_{0};
  int last_published_device_error_ratio_{-1};
  void publish_device_error_ratio_if_enabled();

  // Temperature limits and steps
  float min_temperature_{15.0f};
  float max_temperature_{30.0f};
  float target_temperature_step_{0.5f};
};

}  // namespace olimpia_bridge
}  // namespace esphome
