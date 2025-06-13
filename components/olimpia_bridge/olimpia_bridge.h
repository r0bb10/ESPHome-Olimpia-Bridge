#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/core/preferences.h"
#include "modbus_ascii_handler.h"

namespace esphome {

namespace api {

// Custom API helpers
template <typename T>
T get_execute_arg_value(const ExecuteServiceArgument &arg);

template <typename T>
enums::ServiceArgType to_service_arg_type();

template <>
int get_execute_arg_value<int>(const ExecuteServiceArgument &arg);

template <>
enums::ServiceArgType to_service_arg_type<int>();

}

namespace olimpia_bridge {

// --- Operating modes (EI field in register 101) ---
enum class Mode {
  OFF,        // EI = 00 or unknown
  HEATING,    // Heating mode (EI = 01)
  COOLING,    // Cooling mode (EI = 10)
  AUTO        // Cooling mode (EI = 11)
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
  bool on;               // true if STBY bit is 0 (unit is ON)
  Mode mode;             // EI bits
  FanSpeed fan_speed;    // PRG bits
};

// --- Static helpers for register 101 ---
uint16_t build_command_register(bool on, Mode mode, FanSpeed fan_speed);
ParsedState parse_command_register(uint16_t reg);

// --- Logging and string helpers ---
const char *to_string(Mode mode);
const char *to_string(FanSpeed speed);
void log_parsed_state(const ParsedState &state);

// Forward declare
class OlimpiaBridgeClimate;

// --- OlimpiaBridge component (Modbus bridge + HA services) ---
class OlimpiaBridge : public PollingComponent, public api::CustomAPIDevice {
 public:
  OlimpiaBridge() = default;

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_re_pin(GPIOPin *pin) {
    this->re_pin_ = pin;
    if (this->handler_ != nullptr)
      this->handler_->set_re_pin(pin);
  }

  void set_de_pin(GPIOPin *pin) {
    this->de_pin_ = pin;
    if (this->handler_ != nullptr)
      this->handler_->set_de_pin(pin);
  }

  // Setup bindings
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_ = parent; }
  void set_direction_pin(GPIOPin *pin) { this->direction_pin_ = pin; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_enable_configuration_writes(bool enable) { this->enable_configuration_writes_ = enable; }
  void add_climate(OlimpiaBridgeClimate *climate) { this->climates_.push_back(climate); }

  void increment_modbus_request(bool success);

  // --- Custom HA service to write configuration registers (200–255) ---
  void on_write_register(int address, int reg, int value);

  // --- NEW: Custom HA service to read *any* Modbus register and log it ---
  void on_read_register(int address, int reg);

  // Optional: expose sensor for error ratio
  void set_modbus_error_ratio_sensor(sensor::Sensor *sensor) {
    this->modbus_error_ratio_sensor_ = sensor;
  }

 protected:
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *direction_pin_{nullptr};
  ModbusAsciiHandler *handler_{nullptr};
  std::vector<OlimpiaBridgeClimate *> climates_;

  GPIOPin *re_pin_{nullptr};  // Optional: separate receive enable pin
  GPIOPin *de_pin_{nullptr};  // Optional: separate driver enable pin

  // Enables the HA service to write EEPROM-backed registers
  bool enable_configuration_writes_{false};

  sensor::Sensor *modbus_error_ratio_sensor_{nullptr};
  uint32_t modbus_total_requests_{0};
  uint32_t modbus_failed_requests_{0};
};

// --- OlimpiaBridgeClimate (per slave) ---
class OlimpiaBridgeClimate : public climate::Climate, public Component {
 public:
  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }

  void setup() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  float get_current_temperature() const;
  void control_cycle();

  // Immediately update action (heating/cooling/idle/off) from register 9
  void update_climate_action_from_register9();

  // Update fan speed and request feedback
  void set_fan_mode(const std::string &fan_mode);

  // Update mode and request feedback
  void set_mode(climate::ClimateMode mode);

  // --- Ambient temperature (external sensor via HA) ---
  void set_external_ambient_temperature(float temp);

 protected:
  uint8_t address_{1};
  std::string name_;  // Store and use the entity name
  ModbusAsciiHandler *handler_{nullptr};
  sensor::Sensor *water_temp_sensor_{nullptr};

  uint32_t last_update_time_{0};
  float current_temperature_{0};
  bool initialized_{false};

  // --- Ambient temp from HA with flash fallback ---
  float external_ambient_temperature_{NAN};
  bool has_received_external_temp_{false};
  ESPPreferenceObject ambient_temp_pref_;
  uint32_t last_persist_time_{0};

  // --- Internal control state ---
  bool on_{false};
  Mode mode_{Mode::HEATING};
  FanSpeed fan_speed_{FanSpeed::AUTO};

  // --- Internal register I/O ---
  uint16_t get_status_register();
  void write_control_registers();
  void read_water_temperature();
};

}  // namespace olimpia_bridge
}  // namespace esphome
