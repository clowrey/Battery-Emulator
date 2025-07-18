#include "battery_monitor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <ArduinoJson.h>

namespace esphome {
namespace battery_monitor {

static const char *const TAG = "battery_monitor";

void BatteryMonitorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Battery Monitor...");
  this->last_data_time_ = millis();
}

void BatteryMonitorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Battery Monitor:");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->update_interval_);
  this->check_uart_settings(115200);
}

void BatteryMonitorComponent::loop() {
  this->process_serial_data();
  this->update_connectivity_status();
}

void BatteryMonitorComponent::process_serial_data() {
  while (this->available()) {
    char c = this->read();
    
    if (c == '\n' || c == '\r') {
      if (!this->buffer_.empty()) {
        this->parse_serial_line(this->buffer_);
        this->buffer_.clear();
      }
    } else {
      this->buffer_ += c;
      // Prevent buffer overflow
      if (this->buffer_.length() > 2048) {
        this->buffer_.clear();
      }
    }
  }
}

void BatteryMonitorComponent::parse_serial_line(const std::string &line) {
  ESP_LOGD(TAG, "Received line: %s", line.c_str());
  
  // Parse format: SERIAL_API:<data_type>:<json_message>
  if (line.find("SERIAL_API:") != 0) {
    return; // Not a Serial API message
  }
  
  size_t first_colon = line.find(':', 11); // After "SERIAL_API:"
  if (first_colon == std::string::npos) {
    ESP_LOGW(TAG, "Invalid Serial API format: %s", line.c_str());
    return;
  }
  
  std::string data_type = line.substr(11, first_colon - 11);
  std::string json_data = line.substr(first_colon + 1);
  
  ESP_LOGD(TAG, "Data type: %s, JSON: %s", data_type.c_str(), json_data.c_str());
  
  this->last_data_time_ = millis();
  
  if (data_type == "info") {
    this->handle_info_data(json_data);
  } else if (data_type == "spec_data") {
    this->handle_spec_data(json_data);
  } else if (data_type == "balancing_data") {
    this->handle_balancing_data(json_data);
  } else if (data_type == "events") {
    this->handle_events_data(json_data);
  } else if (data_type == "status") {
    this->handle_status_data(json_data);
  }
}

void BatteryMonitorComponent::handle_info_data(const std::string &json_data) {
  DynamicJsonDocument doc(2048);
  deserializeJson(doc, json_data);
  
  // Main battery data
  if (this->soc_sensor_ && doc.containsKey("SOC")) {
    this->soc_sensor_->publish_state(doc["SOC"].as<float>());
  }
  if (this->soc_real_sensor_ && doc.containsKey("SOC_real")) {
    this->soc_real_sensor_->publish_state(doc["SOC_real"].as<float>());
  }
  if (this->state_of_health_sensor_ && doc.containsKey("state_of_health")) {
    this->state_of_health_sensor_->publish_state(doc["state_of_health"].as<float>());
  }
  if (this->temperature_min_sensor_ && doc.containsKey("temperature_min")) {
    this->temperature_min_sensor_->publish_state(doc["temperature_min"].as<float>());
  }
  if (this->temperature_max_sensor_ && doc.containsKey("temperature_max")) {
    this->temperature_max_sensor_->publish_state(doc["temperature_max"].as<float>());
  }
  if (this->cpu_temp_sensor_ && doc.containsKey("cpu_temp")) {
    this->cpu_temp_sensor_->publish_state(doc["cpu_temp"].as<float>());
  }
  if (this->battery_power_sensor_ && doc.containsKey("stat_batt_power")) {
    this->battery_power_sensor_->publish_state(doc["stat_batt_power"].as<float>());
  }
  if (this->battery_current_sensor_ && doc.containsKey("battery_current")) {
    this->battery_current_sensor_->publish_state(doc["battery_current"].as<float>());
  }
  if (this->battery_voltage_sensor_ && doc.containsKey("battery_voltage")) {
    this->battery_voltage_sensor_->publish_state(doc["battery_voltage"].as<float>());
  }
  if (this->cell_max_voltage_sensor_ && doc.containsKey("cell_max_voltage")) {
    this->cell_max_voltage_sensor_->publish_state(doc["cell_max_voltage"].as<float>());
  }
  if (this->cell_min_voltage_sensor_ && doc.containsKey("cell_min_voltage")) {
    this->cell_min_voltage_sensor_->publish_state(doc["cell_min_voltage"].as<float>());
  }
  if (this->cell_voltage_delta_sensor_ && doc.containsKey("cell_voltage_delta")) {
    this->cell_voltage_delta_sensor_->publish_state(doc["cell_voltage_delta"].as<float>());
  }
  if (this->total_capacity_sensor_ && doc.containsKey("total_capacity")) {
    this->total_capacity_sensor_->publish_state(doc["total_capacity"].as<float>());
  }
  if (this->remaining_capacity_real_sensor_ && doc.containsKey("remaining_capacity_real")) {
    this->remaining_capacity_real_sensor_->publish_state(doc["remaining_capacity_real"].as<float>());
  }
  if (this->remaining_capacity_sensor_ && doc.containsKey("remaining_capacity")) {
    this->remaining_capacity_sensor_->publish_state(doc["remaining_capacity"].as<float>());
  }
  if (this->max_discharge_power_sensor_ && doc.containsKey("max_discharge_power")) {
    this->max_discharge_power_sensor_->publish_state(doc["max_discharge_power"].as<float>());
  }
  if (this->max_charge_power_sensor_ && doc.containsKey("max_charge_power")) {
    this->max_charge_power_sensor_->publish_state(doc["max_charge_power"].as<float>());
  }
  if (this->charged_energy_sensor_ && doc.containsKey("charged_energy")) {
    this->charged_energy_sensor_->publish_state(doc["charged_energy"].as<float>());
  }
  if (this->discharged_energy_sensor_ && doc.containsKey("discharged_energy")) {
    this->discharged_energy_sensor_->publish_state(doc["discharged_energy"].as<float>());
  }
  if (this->balancing_active_cells_sensor_ && doc.containsKey("balancing_active_cells")) {
    this->balancing_active_cells_sensor_->publish_state(doc["balancing_active_cells"].as<float>());
  }
  
  // Second battery data
  if (this->soc_2_sensor_ && doc.containsKey("SOC_2")) {
    this->soc_2_sensor_->publish_state(doc["SOC_2"].as<float>());
  }
  if (this->battery_power_2_sensor_ && doc.containsKey("stat_batt_power_2")) {
    this->battery_power_2_sensor_->publish_state(doc["stat_batt_power_2"].as<float>());
  }
  if (this->battery_voltage_2_sensor_ && doc.containsKey("battery_voltage_2")) {
    this->battery_voltage_2_sensor_->publish_state(doc["battery_voltage_2"].as<float>());
  }
  
  // Text sensors
  if (this->bms_status_sensor_ && doc.containsKey("bms_status")) {
    this->bms_status_sensor_->publish_state(doc["bms_status"].as<std::string>());
  }
  if (this->pause_status_sensor_ && doc.containsKey("pause_status")) {
    this->pause_status_sensor_->publish_state(doc["pause_status"].as<std::string>());
  }
}

void BatteryMonitorComponent::handle_spec_data(const std::string &json_data) {
  DynamicJsonDocument doc(4096);
  deserializeJson(doc, json_data);
  
  if (doc.containsKey("cell_voltages") && doc["cell_voltages"].is<JsonArray>()) {
    JsonArray cell_voltages = doc["cell_voltages"];
    
    for (size_t i = 0; i < cell_voltages.size(); ++i) {
      auto it = this->cell_voltage_sensors_.find(i);
      if (it != this->cell_voltage_sensors_.end() && it->second != nullptr) {
        it->second->publish_state(cell_voltages[i].as<float>());
      }
    }
  }
}

void BatteryMonitorComponent::handle_balancing_data(const std::string &json_data) {
  DynamicJsonDocument doc(2048);
  deserializeJson(doc, json_data);
  
  // This could be used to create individual cell balancing sensors if needed
  ESP_LOGD(TAG, "Balancing data received: %s", json_data.c_str());
}

void BatteryMonitorComponent::handle_events_data(const std::string &json_data) {
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json_data);
  
  if (this->last_event_sensor_ && doc.containsKey("message")) {
    this->last_event_sensor_->publish_state(doc["message"].as<std::string>());
  }
  if (this->event_severity_sensor_ && doc.containsKey("severity")) {
    this->event_severity_sensor_->publish_state(doc["severity"].as<std::string>());
  }
}

void BatteryMonitorComponent::handle_status_data(const std::string &json_data) {
  // Status data is typically just "online" string
  ESP_LOGD(TAG, "Status: %s", json_data.c_str());
}

void BatteryMonitorComponent::update_connectivity_status() {
  uint32_t now = millis();
  bool is_connected = (now - this->last_data_time_) < 15000; // 15 seconds timeout
  
  if (this->emulator_online_sensor_) {
    this->emulator_online_sensor_->publish_state(is_connected);
  }
  
  // Battery connected status could be determined from BMS status or other criteria
  if (this->battery_connected_sensor_) {
    this->battery_connected_sensor_->publish_state(is_connected);
  }
}

}  // namespace battery_monitor
}  // namespace esphome 