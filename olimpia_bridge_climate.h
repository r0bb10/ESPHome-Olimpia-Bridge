#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/preferences.h"
#include "fsm/modbus_ascii_handler.h"

namespace esphome {
namespace olimpia_bridge {

// --- Operating Modes ---
enum class Mode : uint8_t {
  AUTO,
  COOLING,
  HEATING,
  UNKNOWN,
};

// --- Fan Speeds ---
enum class FanSpeed : uint8_t {
  AUTO,
  MIN,
  NIGHT,
  MAX,
  UNKNOWN,
};

// --- Parsed State from Register 101 ---
struct ParsedState {
  bool on = false;
  bool cp = false;
  FanSpeed fan_speed = FanSpeed::UNKNOWN;
  Mode mode = Mode::UNKNOWN;
};

// --- Persisted Climate State ---
struct PersistedClimateState {
  float target_temperature;
  Mode mode;
  FanSpeed fan_speed;
  bool on;
};

// --- OlimpiaBridgeClimate Class ---
class OlimpiaBridgeClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }

 protected:
  void control_cycle();
  void refresh_from_register_101();
  void update_state_from_parsed(const ParsedState &parsed);
  void write_control_registers();
  void read_initial_status_register();
  void read_water_temperature();
  void maybe_save_state();

  uint8_t address_;
  ModbusAsciiHandler *handler_{nullptr};
  sensor::Sensor *water_temp_sensor_{nullptr};

  float target_temperature_{22.0f};
  float current_temperature_{NAN};
  float external_ambient_temperature_{NAN};

  bool has_received_external_temp_{false};
  bool on_{false};
  bool boot_cycle_done_{false};

  Mode mode_{Mode::UNKNOWN};
  FanSpeed fan_speed_{FanSpeed::UNKNOWN};

  PersistedClimateState last_saved_state_;
  ESPPreferenceObject saved_state_pref_;
  ESPPreferenceObject ambient_temp_pref_;

  uint32_t last_state_save_time_{0};
  uint32_t last_persist_time_{0};
  uint32_t last_update_time_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
