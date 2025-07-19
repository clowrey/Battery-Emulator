#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <map>
#include <string>
#include <cstdint>

namespace esphome {
namespace battery_monitor {

class BatteryMonitorComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  // Main battery sensors
  void set_soc_sensor(sensor::Sensor *sensor) { this->soc_sensor_ = sensor; }
  void set_soc_real_sensor(sensor::Sensor *sensor) { this->soc_real_sensor_ = sensor; }
  void set_state_of_health_sensor(sensor::Sensor *sensor) { this->state_of_health_sensor_ = sensor; }
  void set_temperature_min_sensor(sensor::Sensor *sensor) { this->temperature_min_sensor_ = sensor; }
  void set_temperature_max_sensor(sensor::Sensor *sensor) { this->temperature_max_sensor_ = sensor; }
  void set_cpu_temp_sensor(sensor::Sensor *sensor) { this->cpu_temp_sensor_ = sensor; }
  void set_battery_power_sensor(sensor::Sensor *sensor) { this->battery_power_sensor_ = sensor; }
  void set_battery_current_sensor(sensor::Sensor *sensor) { this->battery_current_sensor_ = sensor; }
  void set_battery_voltage_sensor(sensor::Sensor *sensor) { this->battery_voltage_sensor_ = sensor; }
  void set_cell_max_voltage_sensor(sensor::Sensor *sensor) { this->cell_max_voltage_sensor_ = sensor; }
  void set_cell_min_voltage_sensor(sensor::Sensor *sensor) { this->cell_min_voltage_sensor_ = sensor; }
  void set_cell_voltage_delta_sensor(sensor::Sensor *sensor) { this->cell_voltage_delta_sensor_ = sensor; }
  void set_total_capacity_sensor(sensor::Sensor *sensor) { this->total_capacity_sensor_ = sensor; }
  void set_remaining_capacity_real_sensor(sensor::Sensor *sensor) { this->remaining_capacity_real_sensor_ = sensor; }
  void set_remaining_capacity_sensor(sensor::Sensor *sensor) { this->remaining_capacity_sensor_ = sensor; }
  void set_max_discharge_power_sensor(sensor::Sensor *sensor) { this->max_discharge_power_sensor_ = sensor; }
  void set_max_charge_power_sensor(sensor::Sensor *sensor) { this->max_charge_power_sensor_ = sensor; }
  void set_charged_energy_sensor(sensor::Sensor *sensor) { this->charged_energy_sensor_ = sensor; }
  void set_discharged_energy_sensor(sensor::Sensor *sensor) { this->discharged_energy_sensor_ = sensor; }
  void set_balancing_active_cells_sensor(sensor::Sensor *sensor) { this->balancing_active_cells_sensor_ = sensor; }

  // Second battery sensors
  void set_soc_2_sensor(sensor::Sensor *sensor) { this->soc_2_sensor_ = sensor; }
  void set_battery_power_2_sensor(sensor::Sensor *sensor) { this->battery_power_2_sensor_ = sensor; }
  void set_battery_voltage_2_sensor(sensor::Sensor *sensor) { this->battery_voltage_2_sensor_ = sensor; }

  // Text sensors
  void set_bms_status_sensor(text_sensor::TextSensor *sensor) { this->bms_status_sensor_ = sensor; }
  void set_pause_status_sensor(text_sensor::TextSensor *sensor) { this->pause_status_sensor_ = sensor; }
  void set_last_event_sensor(text_sensor::TextSensor *sensor) { this->last_event_sensor_ = sensor; }
  void set_event_severity_sensor(text_sensor::TextSensor *sensor) { this->event_severity_sensor_ = sensor; }

  // Binary sensors
  void set_battery_connected_sensor(binary_sensor::BinarySensor *sensor) { this->battery_connected_sensor_ = sensor; }
  void set_emulator_online_sensor(binary_sensor::BinarySensor *sensor) { this->emulator_online_sensor_ = sensor; }

  // Cell voltage sensors
  void add_cell_voltage_sensor(sensor::Sensor *sensor, int index) { 
    this->cell_voltage_sensors_[index] = sensor; 
  }

 protected:
  uint32_t update_interval_{5000};
  uint32_t last_update_{0};
  uint32_t last_data_time_{0};
  
  std::string buffer_;
  
  // Main battery sensors
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *soc_real_sensor_{nullptr};
  sensor::Sensor *state_of_health_sensor_{nullptr};
  sensor::Sensor *temperature_min_sensor_{nullptr};
  sensor::Sensor *temperature_max_sensor_{nullptr};
  sensor::Sensor *cpu_temp_sensor_{nullptr};
  sensor::Sensor *battery_power_sensor_{nullptr};
  sensor::Sensor *battery_current_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *cell_max_voltage_sensor_{nullptr};
  sensor::Sensor *cell_min_voltage_sensor_{nullptr};
  sensor::Sensor *cell_voltage_delta_sensor_{nullptr};
  sensor::Sensor *total_capacity_sensor_{nullptr};
  sensor::Sensor *remaining_capacity_real_sensor_{nullptr};
  sensor::Sensor *remaining_capacity_sensor_{nullptr};
  sensor::Sensor *max_discharge_power_sensor_{nullptr};
  sensor::Sensor *max_charge_power_sensor_{nullptr};
  sensor::Sensor *charged_energy_sensor_{nullptr};
  sensor::Sensor *discharged_energy_sensor_{nullptr};
  sensor::Sensor *balancing_active_cells_sensor_{nullptr};

  // Second battery sensors
  sensor::Sensor *soc_2_sensor_{nullptr};
  sensor::Sensor *battery_power_2_sensor_{nullptr};
  sensor::Sensor *battery_voltage_2_sensor_{nullptr};

  // Text sensors
  text_sensor::TextSensor *bms_status_sensor_{nullptr};
  text_sensor::TextSensor *pause_status_sensor_{nullptr};
  text_sensor::TextSensor *last_event_sensor_{nullptr};
  text_sensor::TextSensor *event_severity_sensor_{nullptr};

  // Binary sensors
  binary_sensor::BinarySensor *battery_connected_sensor_{nullptr};
  binary_sensor::BinarySensor *emulator_online_sensor_{nullptr};

  // Cell voltage sensors (indexed)
  std::map<int, sensor::Sensor *> cell_voltage_sensors_;

  void process_serial_data();
  void parse_serial_line(const std::string &line);
  void handle_info_data(const std::string &json_data);
  void handle_spec_data(const std::string &json_data);
  void handle_balancing_data(const std::string &json_data);
  void handle_events_data(const std::string &json_data);
  void handle_status_data(const std::string &json_data);
  void update_connectivity_status();
};

}  // namespace battery_monitor
}  // namespace esphome 