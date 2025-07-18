import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from . import battery_monitor_ns, BatteryMonitorComponent

DEPENDENCIES = ["battery_monitor"]

CONF_BMS_STATUS = "bms_status"
CONF_PAUSE_STATUS = "pause_status"
CONF_LAST_EVENT = "last_event"
CONF_EVENT_SEVERITY = "event_severity"

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema().extend(
        {
            cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
            cv.Required(CONF_ID): cv.use_id(BatteryMonitorComponent),
            cv.Optional(CONF_BMS_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PAUSE_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAST_EVENT): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_EVENT_SEVERITY): text_sensor.text_sensor_schema(),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    
    if CONF_BMS_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_BMS_STATUS])
        cg.add(parent.set_bms_status_sensor(sens))
    
    if CONF_PAUSE_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PAUSE_STATUS])
        cg.add(parent.set_pause_status_sensor(sens))
    
    if CONF_LAST_EVENT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_EVENT])
        cg.add(parent.set_last_event_sensor(sens))
    
    if CONF_EVENT_SEVERITY in config:
        sens = await text_sensor.new_text_sensor(config[CONF_EVENT_SEVERITY])
        cg.add(parent.set_event_severity_sensor(sens)) 