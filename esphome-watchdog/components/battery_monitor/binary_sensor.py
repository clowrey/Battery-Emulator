import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_CONNECTIVITY

from . import battery_monitor_ns, BatteryMonitorComponent

DEPENDENCIES = ["battery_monitor"]

CONF_BATTERY_CONNECTED = "battery_connected"
CONF_EMULATOR_ONLINE = "emulator_online"

CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema().extend(
        {
            cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
            cv.Required(CONF_ID): cv.use_id(BatteryMonitorComponent),
            cv.Optional(CONF_BATTERY_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY
            ),
            cv.Optional(CONF_EMULATOR_ONLINE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY
            ),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    
    if CONF_BATTERY_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_BATTERY_CONNECTED])
        cg.add(parent.set_battery_connected_sensor(sens))
    
    if CONF_EMULATOR_ONLINE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_EMULATOR_ONLINE])
        cg.add(parent.set_emulator_online_sensor(sens)) 