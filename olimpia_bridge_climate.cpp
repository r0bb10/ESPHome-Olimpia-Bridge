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

// --- Control Handler ---
void OlimpiaBridgeClimate::control(const climate::ClimateCall &call) {
  ESP_LOGD(TAG, "[%s] Received control call", this->get_name().c_str());
}

// --- Placeholder Methods ---
void OlimpiaBridgeClimate::control_cycle() {}
void OlimpiaBridgeClimate::refresh_from_register_101() {}
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {}
void OlimpiaBridgeClimate::write_control_registers() {}
void OlimpiaBridgeClimate::read_initial_status_register() {}
void OlimpiaBridgeClimate::read_water_temperature() {}
void OlimpiaBridgeClimate::maybe_save_state() {}

}  // namespace olimpia_bridge
}  // namespace esphome
