import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_INDEX,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_ENERGY_STORAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

from . import battery_monitor_ns, BatteryMonitorComponent

DEPENDENCIES = ["battery_monitor"]

CONF_SOC = "soc"
CONF_SOC_REAL = "soc_real"
CONF_SOC_2 = "soc_2"
CONF_STATE_OF_HEALTH = "state_of_health"
CONF_TEMPERATURE_MIN = "temperature_min"
CONF_TEMPERATURE_MAX = "temperature_max"
CONF_CPU_TEMP = "cpu_temp"
CONF_BATTERY_POWER = "battery_power"
CONF_BATTERY_POWER_2 = "battery_power_2"
CONF_BATTERY_CURRENT = "battery_current"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_BATTERY_VOLTAGE_2 = "battery_voltage_2"
CONF_CELL_MAX_VOLTAGE = "cell_max_voltage"
CONF_CELL_MIN_VOLTAGE = "cell_min_voltage"
CONF_CELL_VOLTAGE_DELTA = "cell_voltage_delta"
CONF_TOTAL_CAPACITY = "total_capacity"
CONF_REMAINING_CAPACITY_REAL = "remaining_capacity_real"
CONF_REMAINING_CAPACITY = "remaining_capacity"
CONF_MAX_DISCHARGE_POWER = "max_discharge_power"
CONF_MAX_CHARGE_POWER = "max_charge_power"
CONF_CHARGED_ENERGY = "charged_energy"
CONF_DISCHARGED_ENERGY = "discharged_energy"
CONF_BALANCING_ACTIVE_CELLS = "balancing_active_cells"
CONF_CELL_VOLTAGES = "cell_voltages"

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema().extend(
        {
            cv.GenerateID(): cv.declare_id(sensor.Sensor),
            cv.Required(CONF_ID): cv.use_id(BatteryMonitorComponent),
            cv.Optional(CONF_SOC): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOC_REAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOC_2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_STATE_OF_HEALTH): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_MIN): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_MAX): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CPU_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_POWER_2): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE_2): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CELL_MAX_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CELL_MIN_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CELL_VOLTAGE_DELTA): sensor.sensor_schema(
                unit_of_measurement="mV",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TOTAL_CAPACITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY_STORAGE,
                state_class=STATE_CLASS_TOTAL,
            ),
            cv.Optional(CONF_REMAINING_CAPACITY_REAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY_STORAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REMAINING_CAPACITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY_STORAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MAX_DISCHARGE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MAX_CHARGE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGED_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_DISCHARGED_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_BALANCING_ACTIVE_CELLS): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CELL_VOLTAGES): cv.All(
                cv.ensure_list(
                    cv.Schema({
                        cv.Required(CONF_INDEX): cv.int_range(min=0),
                        **sensor.sensor_schema(
                            unit_of_measurement=UNIT_VOLT,
                            accuracy_decimals=3,
                            device_class=DEVICE_CLASS_VOLTAGE,
                            state_class=STATE_CLASS_MEASUREMENT,
                        ).schema
                    })
                ),
                cv.Length(max=108)  # Tesla Model 3 has up to 108 cells
            ),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    
    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(parent.set_soc_sensor(sens))
    
    if CONF_SOC_REAL in config:
        sens = await sensor.new_sensor(config[CONF_SOC_REAL])
        cg.add(parent.set_soc_real_sensor(sens))
    
    if CONF_SOC_2 in config:
        sens = await sensor.new_sensor(config[CONF_SOC_2])
        cg.add(parent.set_soc_2_sensor(sens))
    
    if CONF_STATE_OF_HEALTH in config:
        sens = await sensor.new_sensor(config[CONF_STATE_OF_HEALTH])
        cg.add(parent.set_state_of_health_sensor(sens))
    
    if CONF_TEMPERATURE_MIN in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_MIN])
        cg.add(parent.set_temperature_min_sensor(sens))
    
    if CONF_TEMPERATURE_MAX in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_MAX])
        cg.add(parent.set_temperature_max_sensor(sens))
    
    if CONF_CPU_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_CPU_TEMP])
        cg.add(parent.set_cpu_temp_sensor(sens))
    
    if CONF_BATTERY_POWER in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_POWER])
        cg.add(parent.set_battery_power_sensor(sens))
    
    if CONF_BATTERY_POWER_2 in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_POWER_2])
        cg.add(parent.set_battery_power_2_sensor(sens))
    
    if CONF_BATTERY_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_CURRENT])
        cg.add(parent.set_battery_current_sensor(sens))
    
    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(parent.set_battery_voltage_sensor(sens))
    
    if CONF_BATTERY_VOLTAGE_2 in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE_2])
        cg.add(parent.set_battery_voltage_2_sensor(sens))
    
    if CONF_CELL_MAX_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_CELL_MAX_VOLTAGE])
        cg.add(parent.set_cell_max_voltage_sensor(sens))
    
    if CONF_CELL_MIN_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_CELL_MIN_VOLTAGE])
        cg.add(parent.set_cell_min_voltage_sensor(sens))
    
    if CONF_CELL_VOLTAGE_DELTA in config:
        sens = await sensor.new_sensor(config[CONF_CELL_VOLTAGE_DELTA])
        cg.add(parent.set_cell_voltage_delta_sensor(sens))
    
    if CONF_TOTAL_CAPACITY in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_CAPACITY])
        cg.add(parent.set_total_capacity_sensor(sens))
    
    if CONF_REMAINING_CAPACITY_REAL in config:
        sens = await sensor.new_sensor(config[CONF_REMAINING_CAPACITY_REAL])
        cg.add(parent.set_remaining_capacity_real_sensor(sens))
    
    if CONF_REMAINING_CAPACITY in config:
        sens = await sensor.new_sensor(config[CONF_REMAINING_CAPACITY])
        cg.add(parent.set_remaining_capacity_sensor(sens))
    
    if CONF_MAX_DISCHARGE_POWER in config:
        sens = await sensor.new_sensor(config[CONF_MAX_DISCHARGE_POWER])
        cg.add(parent.set_max_discharge_power_sensor(sens))
    
    if CONF_MAX_CHARGE_POWER in config:
        sens = await sensor.new_sensor(config[CONF_MAX_CHARGE_POWER])
        cg.add(parent.set_max_charge_power_sensor(sens))
    
    if CONF_CHARGED_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_CHARGED_ENERGY])
        cg.add(parent.set_charged_energy_sensor(sens))
    
    if CONF_DISCHARGED_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_DISCHARGED_ENERGY])
        cg.add(parent.set_discharged_energy_sensor(sens))
    
    if CONF_BALANCING_ACTIVE_CELLS in config:
        sens = await sensor.new_sensor(config[CONF_BALANCING_ACTIVE_CELLS])
        cg.add(parent.set_balancing_active_cells_sensor(sens))
    
    if CONF_CELL_VOLTAGES in config:
        for cell_config in config[CONF_CELL_VOLTAGES]:
            sens = await sensor.new_sensor(cell_config)
            cg.add(parent.add_cell_voltage_sensor(sens, cell_config[CONF_INDEX])) 