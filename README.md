# Olimpia Bridge for ESPHome

**Olimpia Bridge** is a custom [ESPHome](https://esphome.io) component that enables full control of Olimpia Splendid fancoil HVAC units over **Modbus ASCII via UART**.

> This component is based on the excellent original project by [@dumpfheimer](https://github.com/dumpfheimer/olimpia_splendid_bi2_modbus_controller) — huge thanks for the hard work and reverse-engineering he did made the creation of this component possible!

This ESPHome version replicates the same communication behavior, supports multiple slave units, and integrates seamlessly with Home Assistant using its **native API**. Unlike the original project, it does **not use HTTP or MQTT**, resulting in tighter integration and simpler configuration.

## 🚀 Features

- **Modbus-ASCII** communication (RTU-like over ASCII) 
- Communication behavior and timing comply with official Olimpia Splendid specifications & recommendations
- Multiple HVAC slave unit support (independent Modbus addresses)  
- **External ambient temperature** injection (via Home Assistant) with EEPROM fallback    
- Optional **water temperature sensor** per unit 
- Optional **Modbus error ratio** sensor
- Custom HA services:  
  - `olimpia_bridge.write_register` (allows writing to configuration registers for advanced tuning parameters)
  - `olimpia_bridge.read_register` (manual reads for debugging) 

---

## 🔍 How It Works

1. **Initialization (setup)**  
   - Configure UART and RE/DE pins for RS-485 direction control, optionally can be used a single direction_pin if supported

2. **Polling & Control Loop (update)**  
   Every `update_interval` (default 60 s), for each configured climate unit:  
   1. **Write Register 101**  
      - Encodes **fan speed** (bits 0–2), **mode** (bits 2–3), and **standby** (bit 7)  
   2. **Write Register 102**  
      - Sends the **target temperature** (°C × 10)  
   3. **Write Register 103**  
      - Sends the **external ambient temperature** (°C × 10)  
      - Falls back to an EEPROM-persisted value if no HA update has arrived  
   4. **Read Register 1**  
      - Retrieves the **water temperature** (°C × 10) and publishes it to the sensor  

3. **Error Tracking**  
   - Each Modbus transaction calls `increment_modbus_request(success)`  
   - If enabled, publishes “Modbus Error Ratio” sensor as `%` of failed requests  

4. **Ambient-Temp Injection & Persistence**  
   - **Home Assistant sensor** pushes room temperature via a `lambda` calling  
     ```yaml
     id(<unit>).set_external_ambient_temperature(x);
     ```  
   - On first HA update, that value is used immediately and then saved to EEPROM once per 24 h  
   - On reboot or HA sensor loss, the last-saved EEPROM temperature is used for Register 103  

5. **Configuration Writes (optional)**  
   - If `enable_configuration_writes: true`, exposes `write_register` service for known filtered registers (in 200–255 range)
   - Allows on-the-fly tuning of persistent parameters (address, fan-speed limits, etc.)  

---

## 🛠 Installation

### ✅ Option 1: Install via GitHub (Preferred)

This is the cleanest and easiest way to keep your component up-to-date.

**1. Add the component source:**

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/r0bb10/esphome-olimpia-bridge.git
      ref: main
    components: [olimpia_bridge]
```

---

### 🗂 Option 2: Install from Local Folder (Home Assistant folder example)

Use this if you're making modifications or developing locally.

**1. File structure:**

```
homeassistant/
└── esphome/
    └── components/
        └── olimpia_bridge/
            ├── __init__.py
            ├── olimpia_bridge.cpp
            ├── olimpia_bridge.h
            ├── modbus_ascii_handler.cpp
            └── modbus_ascii_handler.h
```

**2. YAML config:**

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [olimpia_bridge]
```

---

## ⚙️ Example Configuration

```yaml
uart:
  id: modbus_uart
  tx_pin: GPIO37 # DI
  rx_pin: GPIO39 # RO
  baud_rate: 9600
  data_bits: 7
  stop_bits: 1
  parity: EVEN

olimpia_bridge:
  uart_id: modbus_uart
  direction_pin: GPIO35 # RE/DE | mandatory if used a single RE/DE or..
  re_pin: GPIO35 # RE - Blue | mandatory if not using direction_pin
  de_pin: GPIO33 # DE - White	| mandatory if not using direction_pin
  enable_configuration_writes: false # optiona, (default to false)
  update_interval: 60s # optional (default 60s)
  modbus_timeout: 200ms # optional (default 200ms)
  modbus_error_ratio_sensor: # optional
    name: "Modbus Error Ratio"
    icon: "mdi:alert-circle"
  climates:
    - id: living_room
      name: Living Room Fancoil
      address: 1 # Modbus address
      water_temperature_sensor: # optional
        name: Living Room Water Temp
    - id: bedroom
      name: Bedroom Unit
      address: 2 # Modbus address
      # no water sensor
```

### 🔄 Ambient Temperature Injection

```yaml
sensor:
  - platform: homeassistant
    id: living_room_temp
    entity_id: sensor.living_room_temp
    on_value:
      then:
        - lambda: |-
            id(living_room).set_external_ambient_temperature(x);

  - platform: homeassistant
    id: bedroom_temp
    entity_id: sensor.bedroom_temp
    on_value:
      then:
        - lambda: |-
            id(bedroom).set_external_ambient_temperature(x);
```

### 🏷️ Services

- **olimpia_bridge.write_register**
Write a holding register, filtered to only the permittet to avoid potential bricks.

```yaml
- service: olimpia_bridge.write_register
  data:
    address: 1
    register: 210
    value: 680
```

- **olimpia_bridge.read_register**
Read a single register.

```yaml
- service: olimpia_bridge.read_register
  data:
    address: 1
    register: 103
```

---

## 📥 Input / Measured Registers

| Register | Description                                      | Access | Notes                                               |
|----------|--------------------------------------------------|--------|-----------------------------------------------------|
| 1        | Water temperature                                | R      | Reported by each slave, optional sensor             |
| 9        | Valve and system status bitfield                 | R      | Bit 13 = ev1, 14 = boiler, 15 = chiller             |
| 101      | HVAC status bits                                 | R/W    | Controls power, mode, and fan                       |
| 102      | Setpoint temperature (°C × 10)                   | R/W    | Desired setpoint temperature                        |
| 103      | External ambient temperature (°C × 10)           | R/W    | Written from External Home Assistant sensor         |

---

## 🧰 Persistent Configuration Registers

These registers should not be written frequently to avoid EEPROM wear. Only use them for configuration tasks.

| Register | Description                                      | Access | Notes                                               |
|----------|--------------------------------------------------|--------|-----------------------------------------------------|
| 200      | Modbus slave address                             | R/W    | Must be unique; avoid overwrites                    |
| 202      | Minimum allowed setpoint (e.g. 15°C)             | R/W    | Default: 15°C                                       |
| 203      | Maximum allowed setpoint (e.g. 35°C)             | R/W    | Default: 35°C                                       |
| 210      | Min fan speed in cool mode (e.g. 680)            | R/W    | Unit: RPM                                           |
| 211      | Min fan speed in heat mode (e.g. 680)            | R/W    | Unit: RPM                                           |
| 212      | Max fan speed in cool mode (e.g. 950)            | R/W    | Unit: RPM                                           |
| 213      | Max fan speed in heat mode (e.g. 950)            | R/W    | Unit: RPM                                           |
| 214      | Max fan speed in min mode (e.g. 680)             | R/W    | Unit: RPM                                           |
| 215      | Max fan speed in night mode (e.g. 680)           | R/W    | Unit: RPM                                           |
| 217      | Minimum water temperature for heating (°C)       | R/W    | Default: 35°C                                       |
| 219      | Maximum water temperature for cooling (°C)       | R/W    | Default: 20°C                                       |

---

## 🛡️ Configuration Safety

Registers 200–255 are persistent EEPROM registers and must be written sparingly.  
To expose a Home Assistant service (`olimpia_write_register`) that allows this, set:

```yaml
enable_configuration_writes: true
```

---

## ⚠️ Notes

- Ensure Olimpia Splendid units use Modbus ASCII and follow standard register map.
- The `101 → 102 → 103` sequence is written cyclically every 60s to meet controller expectations.
- Component ensures proper UART timing, buffer management, and frame parsing as per original controller.
