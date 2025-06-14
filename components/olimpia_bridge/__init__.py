import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart, sensor, climate
from esphome.const import (
    CONF_ID, CONF_UART_ID, CONF_NAME, CONF_ADDRESS, CONF_UPDATE_INTERVAL
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "climate"]

olimpia_bridge_ns = cg.esphome_ns.namespace("olimpia_bridge")
OlimpiaBridge = olimpia_bridge_ns.class_("OlimpiaBridge", cg.Component)
OlimpiaBridgeClimate = olimpia_bridge_ns.class_("OlimpiaBridgeClimate", climate.Climate, cg.Component)
ModbusAsciiHandler = olimpia_bridge_ns.class_("ModbusAsciiHandler")

CONF_DIRECTION_PIN = "direction_pin"
CONF_ENABLE_CONFIGURATION_WRITES = "enable_configuration_writes"
CONF_CLIMATES = "climates"
CONF_WATER_TEMPERATURE_SENSOR = "water_temperature_sensor"
CONF_MODBUS_TIMEOUT = "modbus_timeout"
CONF_HANDLER_ID = "handler"
CONF_MODBUS_ERROR_RATIO_SENSOR = "modbus_error_ratio_sensor"

olimpia_bridge_climate_schema = climate.climate_schema(OlimpiaBridgeClimate).extend({
    cv.GenerateID(): cv.declare_id(OlimpiaBridgeClimate),
    cv.Required(CONF_NAME): cv.string,  # Ensure this line correctly passes through
    cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=31),
    cv.Optional(CONF_WATER_TEMPERATURE_SENSOR): sensor.sensor_schema(
        unit_of_measurement="°C", accuracy_decimals=1
    ),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OlimpiaBridge),
    cv.GenerateID(CONF_HANDLER_ID): cv.declare_id(ModbusAsciiHandler),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

    # Allow either 'direction_pin' or both 're_pin' and 'de_pin'
    cv.Optional(CONF_DIRECTION_PIN): pins.gpio_output_pin_schema,
    cv.Optional("re_pin"): pins.gpio_output_pin_schema,
    cv.Optional("de_pin"): pins.gpio_output_pin_schema,

    cv.Optional(CONF_ENABLE_CONFIGURATION_WRITES, default=False): cv.boolean,
    cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
    cv.Optional(CONF_MODBUS_TIMEOUT, default="200ms"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_MODBUS_ERROR_RATIO_SENSOR): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=2,
        icon="mdi:alert-circle"
    ),
    cv.Required(CONF_CLIMATES): cv.ensure_list(olimpia_bridge_climate_schema),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    controller = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(controller, config)

    uart_var = await cg.get_variable(config[CONF_UART_ID])
    cg.add(controller.set_uart_parent(uart_var))

    if CONF_DIRECTION_PIN in config:
        direction_pin = await cg.gpio_pin_expression(config[CONF_DIRECTION_PIN])
        cg.add(controller.set_direction_pin(direction_pin))
    elif "re_pin" in config and "de_pin" in config:
        re_pin = await cg.gpio_pin_expression(config["re_pin"])
        de_pin = await cg.gpio_pin_expression(config["de_pin"])
        cg.add(controller.set_re_pin(re_pin))
        cg.add(controller.set_de_pin(de_pin))
    else:
        cg.add_library("esphome", None)  # ensure build doesn't fail
        cg.logger.error("You must specify either 'direction_pin' or both 're_pin' and 'de_pin'.")

    # PATCH: Enable config writes if set in YAML
    if CONF_ENABLE_CONFIGURATION_WRITES in config:
        cg.add(controller.set_enable_configuration_writes(config[CONF_ENABLE_CONFIGURATION_WRITES]))

    cg.add(controller.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    handler = cg.new_Pvariable(config[CONF_HANDLER_ID])
    cg.add(controller.set_handler(handler))
    cg.add(handler.set_bridge(controller))

    if CONF_MODBUS_TIMEOUT in config:
        timeout_ms = config[CONF_MODBUS_TIMEOUT].total_milliseconds
        cg.add(handler.set_timeout(timeout_ms))

    if CONF_MODBUS_ERROR_RATIO_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_MODBUS_ERROR_RATIO_SENSOR])
        cg.add(controller.set_modbus_error_ratio_sensor(sens))

    for climate_conf in config[CONF_CLIMATES]:
        climate_var = cg.new_Pvariable(climate_conf[CONF_ID])
        
        # Explicitly set the climate name from YAML to ESPHome climate
        await climate.register_climate(climate_var, climate_conf)

        # Explicitly set additional properties
        cg.add(climate_var.set_address(climate_conf[CONF_ADDRESS]))

        # Override default object ID (entity_id)
        cg.add(climate_var.set_object_id(climate_conf[CONF_NAME].lower().replace(" ", "_")))

        cg.add(controller.add_climate(climate_var))

        if CONF_WATER_TEMPERATURE_SENSOR in climate_conf:
            sens = await sensor.new_sensor(climate_conf[CONF_WATER_TEMPERATURE_SENSOR])
            cg.add(climate_var.set_water_temp_sensor(sens))
