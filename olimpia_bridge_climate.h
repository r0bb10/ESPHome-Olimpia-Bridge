#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "fsm/modbus_ascii_handler.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace olimpia_bridge {

// --- Operating modes (EI field in register 101) ---
enum class Mode : uint8_t {
  AUTO     = 0,
  COOLING  = 1,
  HEATING  = 2,
  UNKNOWN  = 0xFF
};

// --- Fan speed levels per PRG field (bits 0–2) ---
enum class FanSpeed : uint8_t {
  AUTO    = 0b000,
  MIN     = 0b001,
  NIGHT   = 0b010,
  MAX     = 0b011,
  UNKNOWN = 0xFF
};

// --- Parsed register 101 state ---
struct ParsedState {
  bool on = false;
  bool cp = false;
  FanSpeed fan_speed = FanSpeed::UNKNOWN;
  uint8_t fan_speed_raw = 0;
  Mode mode = Mode::UNKNOWN;
};

// --- OlimpiaBridgeClimate class ---
class OlimpiaBridgeClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }
  void set_external_ambient_temperature(float temp);

  void refresh_from_register_101();
  void control_cycle();
  void read_water_temperature();

 protected:
  void update_state_from_parsed(const ParsedState &parsed);
  void write_control_registers_cycle();
  uint16_t get_status_register();

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

  uint32_t last_update_time_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
