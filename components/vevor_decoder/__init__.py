"""Vevor 7-in-1 Weather Station decoder for ESPHome."""

import logging

from esphome import final_validate as fv
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, remote_base, remote_receiver, sensor
from esphome.core import CORE
from esphome.const import (
    __version__ as ESPHOME_VERSION,
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_ILLUMINANCE,
    DEVICE_CLASS_PRECIPITATION,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WIND_SPEED,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_DEGREES,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_LUX,
    UNIT_MILLIMETER,
    UNIT_PERCENT,
)

try:
    from esphome.components import cc1101
except ImportError:  # ESPHome without a cc1101 component
    cc1101 = None

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@parrel"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

# Packet mode needs the cc1101 listener interface (ESPHome 2026.2).
HAS_CC1101_LISTENER = cc1101 is not None and cv.Version.parse(
    ESPHOME_VERSION
) >= cv.Version(2026, 2, 0)

vevor_ns = cg.esphome_ns.namespace("vevor_decoder")
VevorDecoder = vevor_ns.class_(
    "VevorDecoder",
    cg.Component,
    remote_base.RemoteReceiverDumperBase,
)

CONF_RECEIVER_ID = "receiver_id"
CONF_CC1101_ID = "cc1101_id"
CONF_SENSOR_ID = "sensor_id"
CONF_BIT_PERIOD = "bit_period"
CONF_RAIN_HOLD = "rain_hold"
CONF_ILLUMINANCE_FILTER = "illuminance_filter"

CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
CONF_WIND_SPEED = "wind_speed"
CONF_WIND_GUST = "wind_gust"
CONF_WIND_DIRECTION = "wind_direction"
CONF_RAIN = "rain"
CONF_UV_INDEX = "uv_index"
CONF_ILLUMINANCE = "illuminance"
CONF_BATTERY_LOW = "battery_low"

# Accepts 0x1A2B as well as a plain integer; the station id is 16 bits.
SENSOR_ID_SCHEMA = cv.All(cv.hex_int, cv.Range(min=0, max=0xFFFF))

FRAME_BYTES = 21
SYNC_WORDS = {(0xCA, 0x54), (0x35, 0xAB)}  # normal and inverted polarity
MEASURED_SYMBOL_RATE = 11260


def _cc1101_id(value):
    if not HAS_CC1101_LISTENER:
        raise cv.Invalid(
            "cc1101_id (packet mode) needs ESPHome 2026.2 or newer; "
            "update ESPHome or use receiver_id with remote_receiver instead"
        )
    return cv.use_id(cc1101.CC1101Component)(value)


def _validate_input(config):
    if CONF_CC1101_ID in config and CONF_RECEIVER_ID in config:
        raise cv.Invalid(
            "Use either cc1101_id (packet mode) or receiver_id "
            "(remote_receiver), not both"
        )
    if CONF_CC1101_ID in config or CONF_RECEIVER_ID in config:
        return config
    # Prefer remote_receiver so existing configurations keep working.
    if (
        "remote_receiver" not in CORE.loaded_integrations
        and "cc1101" in CORE.loaded_integrations
    ):
        config[CONF_CC1101_ID] = _cc1101_id(None)
    else:
        config[CONF_RECEIVER_ID] = cv.use_id(remote_receiver.RemoteReceiverComponent)(
            None
        )
    return config


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(VevorDecoder),
        cv.Optional(CONF_RECEIVER_ID): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
        cv.Optional(CONF_CC1101_ID): _cc1101_id,
        cv.Optional(CONF_SENSOR_ID): SENSOR_ID_SCHEMA,
        cv.Optional(CONF_BIT_PERIOD, default="90us"): cv.All(
            cv.positive_time_period_microseconds,
            cv.Range(min=cv.TimePeriod(microseconds=10)),
        ),
        cv.Optional(CONF_RAIN_HOLD, default=True): cv.boolean,
        cv.Optional(CONF_ILLUMINANCE_FILTER, default=True): cv.boolean,
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WIND_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_GUST): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WIND_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_DIRECTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=0,
            icon="mdi:compass-outline",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAIN): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRECIPITATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
            accuracy_decimals=0,
            icon="mdi:sun-wireless",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)
CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _validate_input)


def _check_radio(config):
    """Reject a cc1101 block that cannot deliver Vevor frames; these fail silently."""
    full = fv.full_config.get()
    radio = full.get_config_for_path(full.get_path_for_id(config[CONF_CC1101_ID])[:-1])
    name = config[CONF_CC1101_ID].id

    def need(ok, message):
        if not ok:
            raise cv.Invalid(f"cc1101 '{name}': {message}")

    need(radio.get("packet_mode", False), "set packet_mode: true for cc1101_id")
    need(
        radio.get("packet_length") == FRAME_BYTES,
        f"set packet_length: {FRAME_BYTES}, the Vevor frame length",
    )
    sync = (radio.get("sync1"), radio.get("sync0"))
    need(
        sync in SYNC_WORDS,
        "set sync1: 0xCA and sync0: 0x54, the Vevor sync word "
        "(or 0x35 / 0xAB if your radio demodulates inverted)",
    )
    need(str(radio.get("sync_mode")) != "None", "sync_mode must not be None")
    need(
        str(radio.get("modulation_type")) in ("2-FSK", "GFSK"),
        "set modulation_type: 2-FSK",
    )
    need(not radio.get("manchester", False), "manchester must be off")
    need(not radio.get("crc_enable", False), "crc_enable must be off")
    need(not radio.get("whitening", False), "whitening must be off")

    # Not fatal, but frames arrive corrupted.
    if radio.get("foc_bs_cs_gate", True):
        _LOGGER.warning(
            "cc1101 '%s': set foc_bs_cs_gate: false. With the gate on, frequency "
            "offset compensation and clock recovery stay frozen and Vevor frames "
            "arrive with slipped bits",
            name,
        )
    rate = radio.get("symbol_rate")
    if rate is not None and abs(rate - MEASURED_SYMBOL_RATE) > 100:
        _LOGGER.warning(
            "cc1101 '%s': symbol_rate %s is off the station's ~%d baud; in packet "
            "mode that slips bits. Use symbol_rate: %d",
            name,
            rate,
            MEASURED_SYMBOL_RATE,
            MEASURED_SYMBOL_RATE,
        )


def _final_validate(config):
    for conf in config if isinstance(config, list) else [config]:
        if CONF_CC1101_ID in conf:
            _check_radio(conf)
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

SENSORS = {
    CONF_TEMPERATURE: "set_temperature_sensor",
    CONF_HUMIDITY: "set_humidity_sensor",
    CONF_WIND_SPEED: "set_wind_speed_sensor",
    CONF_WIND_GUST: "set_wind_gust_sensor",
    CONF_WIND_DIRECTION: "set_wind_direction_sensor",
    CONF_RAIN: "set_rain_sensor",
    CONF_UV_INDEX: "set_uv_index_sensor",
    CONF_ILLUMINANCE: "set_illuminance_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_CC1101_ID in config:
        cg.add_define("USE_VEVOR_DECODER_CC1101")
        radio = await cg.get_variable(config[CONF_CC1101_ID])
        cg.add(radio.register_listener(var))
        cg.add(var.set_packet_input(True))
    else:
        cg.add_define("USE_VEVOR_DECODER_REMOTE_RECEIVER")
        receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
        if hasattr(remote_base, "add_dumper"):
            remote_base.add_dumper(receiver, var)
        else:
            cg.add(receiver.register_dumper(var))

    # -1 is the "accept any station" sentinel on the C++ side.
    cg.add(var.set_sensor_id(config.get(CONF_SENSOR_ID, -1)))
    cg.add(var.set_bit_period(config[CONF_BIT_PERIOD]))
    cg.add(var.set_rain_hold(config[CONF_RAIN_HOLD]))
    cg.add(var.set_illuminance_filter(config[CONF_ILLUMINANCE_FILTER]))

    for key, setter in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    if CONF_BATTERY_LOW in config:
        bin_sens = await binary_sensor.new_binary_sensor(config[CONF_BATTERY_LOW])
        cg.add(var.set_battery_low_binary_sensor(bin_sens))
