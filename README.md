![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-blue?logo=esphome)
![License](https://img.shields.io/github/license/r0bb10/ESPHome-Olimpia-Bridge)
![GitHub stars](https://img.shields.io/github/stars/r0bb10/ESPHome-Olimpia-Bridge?style=social)

# ESPHome Olimpia Bridge

ESPHome custom component for controlling Olimpia Splendid HVAC units via Modbus ASCII over RS-485.

> Based on [@dumpfheimer](https://github.com/dumpfheimer/olimpia_splendid_bi2_modbus_controller)'s work, who not only reverse-engineered the protocol but also created the first standalone controller with Home Assistant integration. Thank you!

## Features

- Full climate control (temperature, fan speed, modes)
- Multi-zone support with independent control
- External temperature injection from Home Assistant sensors
- Optional EMA filtering for temperature stability
- State persistence across reboots
- Water temperature monitoring per unit
- Per-device error ratio tracking
- Virtual presets (Auto/Manual) for advanced automations
- Home Assistant services for direct register access

## Hardware Requirements

**Recommended:**
- ESP32 (any variant)
- MAX3485 RS-485 transceiver (3.3V compatible)

**Supported:**
- ESP8266
- MAX485 RS-485 transceiver (5V, use with level shifter for 3.3V boards)

**Required:**
- Olimpia Splendid unit with B0872 Modbus Interface

> **Note:** Both MAX485 and MAX3485 support single-pin mode by tying DE and RE together (bridge pads or connect externally). MAX3485 is recommended because it operates at 3.3V natively, matching ESP32/ESP8266 logic levels without needing level shifters.

## Wiring

**RS-485 Module → ESP (Recommended - Single Pin):**
- `DI (TX)` → ESP TX pin
- `RO (RX)` → ESP RX pin
- `DE + RE` → ESP GPIO pin (tie together, direction control)
- `VCC` → 3.3V or 5V
- `GND` → GND

**RS-485 Module → HVAC:**
- `A` → HVAC A
- `B` → HVAC B

Invert TX/RX signals in software (see configuration).

## Installation

Add to your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/r0bb10/esphome-olimpia-bridge.git
      ref: main
    components: [olimpia_bridge]
```

## Basic Configuration

### ESP32 (Recommended)

```yaml
esphome:
  name: olimpia-bridge

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    version: recommended

uart:
  id: modbus_uart
  tx_pin:
    number: GPIO1
    inverted: true
  rx_pin:
    number: GPIO3
    inverted: true
  baud_rate: 9600
  data_bits: 7
  stop_bits: 1
  parity: EVEN

olimpia_bridge:
  id: modbus_ascii_bridge
  uart_id: modbus_uart
  en_pin: GPIO5  # Direction control (HIGH=TX, LOW=RX)
  error_ratio_sensor:
    name: "Modbus Error Ratio"
  climates:
    - name: "Living Room AC"
      id: living_room
      address: 1

sensor:
  - platform: homeassistant
    id: living_temp
    entity_id: sensor.living_room_temperature
    on_value:
      - lambda: id(living_room).set_external_ambient_temperature(x);
```

### ESP8266

```yaml
esphome:
  name: olimpia-bridge

esp8266:
  board: d1_mini
  framework:
    version: recommended

# ... rest same as ESP32 config
```

## Advanced Configuration

### Multiple Units

```yaml
olimpia_bridge:
  id: modbus_ascii_bridge
  uart_id: modbus_uart
  en_pin: GPIO5
  activity_pin: GPIO2  # Optional: LED blinks during Modbus communication
  error_ratio_sensor:
    name: "Modbus Error Ratio"
  use_ema: true  # Enable EMA filtering (default: true)
  climates:
    - name: "Living Room AC"
      id: living_room
      address: 1
      ema_alpha: 0.25  # Smoothing factor (0.0-1.0, default: 0.2)
      water_temperature_sensor:
        name: "Living Water Temp"
      device_error_ratio_sensor:
        name: "Living Error Ratio"
      enable_virtual_presets: true
      min_temperature: 16.0
      max_temperature: 28.0
      target_temperature_step: 0.1

    - name: "Bedroom AC"
      id: bedroom
      address: 2
      ema_alpha: 0.1  # More aggressive smoothing
```

## Supported Configurations

### Separate RE/DE Pins (MAX485)

If using MAX485 or similar modules without tied EN pin:

```yaml
olimpia_bridge:
  id: modbus_ascii_bridge
  uart_id: modbus_uart
  re_pin: GPIO35  # Receive Enable (active LOW)
  de_pin: GPIO33  # Driver Enable (active HIGH)
  # ... rest of config
```

## Home Assistant Actions

```yaml
api:
  services:
    - service: read_register
      variables:
        address: int
        reg: int
      then:
        - lambda: id(modbus_ascii_bridge).read_register(address, reg);

    - service: write_register
      variables:
        address: int
        reg: int
        value: int
      then:
        - lambda: id(modbus_ascii_bridge).write_register(address, reg, value);

    - service: dump_configuration
      variables:
        address: int
      then:
        - lambda: id(modbus_ascii_bridge).dump_configuration(address);
```

Call from Home Assistant:

```yaml
# Read a register
action: esphome.olimpia_bridge_read_register
data:
  address: 1
  reg: 103

# Write a register
action: esphome.olimpia_bridge_write_register
data:
  address: 1
  reg: 200
  value: 5

# Dump all configuration registers
action: esphome.olimpia_bridge_dump_configuration
data:
  address: 1
```

## Configuration Options

### `olimpia_bridge`

| Option | Type | Required | Default | Description |
|--------|------|----------|---------|-------------|
| `uart_id` | ID | Yes | - | UART component ID |
| `en_pin` | Pin | Yes* | - | Single pin for RE+DE (active HIGH for TX) |
| `re_pin` | Pin | Yes* | - | Separate RE pin (active LOW) |
| `de_pin` | Pin | Yes* | - | Separate DE pin (active HIGH) |
| `activity_pin` | Pin | No | - | Optional LED that blinks during communication |
| `error_ratio_sensor` | Sensor | Yes | - | Global error ratio sensor |
| `use_ema` | bool | No | `true` | Enable EMA temperature filtering |
| `climates` | List | Yes | - | List of climate entities |

*Either `en_pin` OR both `re_pin`+`de_pin` required.

### Climate Entity Options

| Option | Type | Required | Default | Description |
|--------|------|----------|---------|-------------|
| `name` | string | Yes | - | Display name |
| `id` | ID | Yes | - | Entity ID |
| `address` | int | Yes | - | Modbus address (1-247) |
| `device_id` | ID | No | - | Home Assistant device ID |
| `ema_alpha` | float | No | `0.2` | EMA smoothing factor (0.0-1.0) |
| `water_temperature_sensor` | Sensor | No | - | Water temp sensor |
| `device_error_ratio_sensor` | Sensor | No | - | Per-device error ratio |
| `enable_virtual_presets` | bool | No | `false` | Enable virtual Auto/Manual presets |
| `disable_mode_auto` | bool | No | `false` | Hide AUTO mode in Home Assistant |
| `disable_fan_quiet` | bool | No | `false` | Hide QUIET fan mode |
| `min_temperature` | float | No | `15.0` | Minimum target temperature |
| `max_temperature` | float | No | `30.0` | Maximum target temperature |
| `target_temperature_step` | float | No | `0.5` | Temperature adjustment step |

## EMA Filtering

Exponential Moving Average (EMA) filtering smooths temperature readings from Home Assistant sensors before sending them to the HVAC unit. This prevents the unit from reacting to momentary temperature spikes or sensor noise.

### How It Works

Each new temperature reading is blended with the previous filtered value:
```
filtered_temp = (alpha × new_reading) + ((1 - alpha) × previous_filtered)
```

### Alpha Values

- **`ema_alpha`**: Controls smoothing strength (0.0 - 1.0)
  - `0.1`: Heavy smoothing - very stable but slow to react to real changes
  - `0.2`: Balanced (default) - good stability with reasonable responsiveness
  - `0.3-0.5`: Light smoothing - faster response but less noise rejection
  - `1.0`: No smoothing - uses raw sensor values

### When to Adjust

**Lower alpha (more smoothing):**
- Noisy temperature sensors
- Sensors in locations with rapid air movement
- Units prone to short-cycling

**Higher alpha (less smoothing):**
- High-quality sensors with stable readings
- Need faster response to temperature changes
- Rooms with rapid temperature swings

### Additional Features

- **Trend validation**: Ignores random spikes that don't match the temperature trend
- **Noise rejection**: Filters out readings that deviate significantly from recent values
- **Inactivity reset**: Automatically recalibrates after prolonged sensor inactivity

Disable globally with `use_ema: false` to use raw sensor values.

## Virtual Presets

Virtual presets are automation control flags that don't change how the HVAC operates - they signal your Home Assistant automations to enable or skip climate control for specific zones.

### How They Work

When `enable_virtual_presets: true`, two presets appear in Home Assistant:

- **Auto**: Zone is controlled by your automations
- **Manual**: Zone is excluded from automation control

The HVAC unit operates normally in both modes. The preset only affects whether your automations modify the climate settings.

### Practical Use Cases

**Scenario 1: Per-Room Automation Override**
```yaml
automation:
  - alias: "Smart Climate Control"
    trigger:
      - platform: time_pattern
        minutes: "/5"
    action:
      - repeat:
          for_each:
            - climate.living_room_ac
            - climate.bedroom_ac
          sequence:
            - condition: template
              value_template: "{{ state_attr(repeat.item, 'preset_mode') == 'auto' }}"
            - service: climate.set_temperature
              target:
                entity_id: "{{ repeat.item }}"
              data:
                temperature: "{{ states('input_number.target_temp') }}"
```
Rooms in "Manual" preset are skipped. Switch a room to "Manual" when you want temporary local control.

**Scenario 2: Occupancy-Based Control**
```yaml
automation:
  - alias: "Bedroom Night Mode"
    trigger:
      - platform: state
        entity_id: binary_sensor.bedroom_occupied
        to: "on"
    condition:
      - condition: state
        entity_id: climate.bedroom_ac
        attribute: preset_mode
        state: "auto"
    action:
      - service: climate.set_temperature
        target:
          entity_id: climate.bedroom_ac
        data:
          temperature: 21
```
Only applies if preset is "Auto". When guests use the room, switch to "Manual" to prevent automation interference.

**Scenario 3: Time-Based Zones**
```yaml
automation:
  - alias: "Daytime Living Area Control"
    trigger:
      - platform: time
        at: "08:00:00"
    action:
      - service: climate.set_preset_mode
        target:
          entity_id:
            - climate.living_room_ac
            - climate.kitchen_ac
        data:
          preset_mode: "auto"
      - service: climate.set_preset_mode
        target:
          entity_id: climate.bedroom_ac
        data:
          preset_mode: "manual"
```
Enable automation for active areas during the day, disable for sleeping areas.

### When to Use

- You have complex automation logic that shouldn't affect all zones simultaneously
- Need per-room override without disabling entire automations
- Want users to temporarily exclude specific rooms from automation control
- Building zone-based scheduling systems

Set `enable_virtual_presets: false` (default) if you don't need automation control flags.

## Supported Platforms

- ✅ ESP32 (all variants: S2, S3, C3, etc.)
- ✅ ESP8266
- ✅ ESP32 with Ethernet (e.g., WT32-ETH01)

## Development

See [DEVELOPMENT.md](DEVELOPMENT.md) for register documentation and protocol details.

## License

MIT License - see [LICENSE](LICENSE)
