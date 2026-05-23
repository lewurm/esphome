import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_CALIBRATION,
    CONF_CO2,
    CONF_ID,
    CONF_STATUS,
    DEVICE_CLASS_CARBON_DIOXIDE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PARTS_PER_MILLION,
)

CODEOWNERS = ["@tylermenezes"]
DEPENDENCIES = ["uart"]

CONF_ERROR = "error"
CONF_WARMUP = "warmup"
CONF_IDLE = "idle"
CONF_SELF_TEST = "self_test"
CONF_SERVICE_MODE = "service_mode"
CONF_ELEVATION = "elevation"

UNIT_FOOT = "ft"

t6615_ns = cg.esphome_ns.namespace("t6615")
T6615Component = t6615_ns.class_("T6615Component", cg.PollingComponent, uart.UARTDevice)


def diagnostic_bit_schema():
    return sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(T6615Component),
            cv.Optional(CONF_CO2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CARBON_DIOXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_STATUS): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_ERROR): diagnostic_bit_schema(),
            cv.Optional(CONF_WARMUP): diagnostic_bit_schema(),
            cv.Optional(CONF_CALIBRATION): diagnostic_bit_schema(),
            cv.Optional(CONF_IDLE): diagnostic_bit_schema(),
            cv.Optional(CONF_SELF_TEST): diagnostic_bit_schema(),
            cv.Optional(CONF_SERVICE_MODE): diagnostic_bit_schema(),
            cv.Optional(CONF_ELEVATION): sensor.sensor_schema(
                unit_of_measurement=UNIT_FOOT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "t6615", baud_rate=19200, require_rx=True, require_tx=True
)


_SETTERS = {
    CONF_CO2: "set_co2_sensor",
    CONF_STATUS: "set_status_sensor",
    CONF_ERROR: "set_error_sensor",
    CONF_WARMUP: "set_warmup_sensor",
    CONF_CALIBRATION: "set_calibration_sensor",
    CONF_IDLE: "set_idle_sensor",
    CONF_SELF_TEST: "set_self_test_sensor",
    CONF_SERVICE_MODE: "set_service_mode_sensor",
    CONF_ELEVATION: "set_elevation_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for key, setter in _SETTERS.items():
        if sub := config.get(key):
            sens = await sensor.new_sensor(sub)
            cg.add(getattr(var, setter)(sens))
