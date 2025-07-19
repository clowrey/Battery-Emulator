#include "battery_monitor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <ArduinoJson.h>

namespace esphome {
namespace battery_monitor {

static const char *const TAG = "battery_monitor";

/**
 * CRC16 Implementation for Serial API Data Verification
 * 
 * This implementation matches the CRC calculation in serial_api.cpp to verify
 * data integrity of transmitted cell voltage and balancing data.
 * 
 * Uses CRC-CCITT polynomial (0x1021) with initial value 0xFFFF
 * Detects transmission errors, corruption, and data mix-ups
 */

// CRC16 calculation for data integrity verification (same as serial_api.cpp)
static uint16_t calculate_crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  const uint16_t polynomial = 0x1021; // CRC-CCITT polynomial
  
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)(data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Verify CRC for cell voltage data
static bool verify_cell_voltage_crc(const JsonArray &cell_data, uint16_t expected_crc, size_t num_cells) {
  if (cell_data.size() != num_cells) {
    return false;
  }
  
  // Create raw voltage array for CRC calculation (same format as sender)
  std::vector<uint16_t> voltages_mv(num_cells);
  for (size_t i = 0; i < num_cells; i++) {
    if (i < cell_data.size() && cell_data[i].containsKey("voltage")) {
      float voltage_v = cell_data[i]["voltage"].as<float>();
      voltages_mv[i] = static_cast<uint16_t>(voltage_v * 1000.0f); // Convert back to mV
    } else {
      return false; // Missing data
    }
  }
  
  uint16_t calculated_crc = calculate_crc16((const uint8_t*)voltages_mv.data(), num_cells * sizeof(uint16_t));
  return calculated_crc == expected_crc;
}

// Verify CRC for cell balancing data
static bool verify_cell_balancing_crc(const JsonArray &cell_data, uint16_t expected_crc, size_t num_cells) {
  if (cell_data.size() != num_cells) {
    return false;
  }
  
  // Create raw balancing array for CRC calculation (same format as sender)
  // Note: std::vector<bool> is specialized, so we use a regular array of bool
  // and cast to uint8_t* for CRC calculation (matching sender behavior)
  std::vector<bool> balancing_status(num_cells);
  for (size_t i = 0; i < num_cells; i++) {
    if (i < cell_data.size() && cell_data[i].containsKey("balancing")) {
      balancing_status[i] = cell_data[i]["balancing"].as<bool>();
    } else {
      return false; // Missing data
    }
  }
  
  // Convert to byte array to match sender's CRC calculation
  std::vector<uint8_t> balancing_bytes(num_cells);
  for (size_t i = 0; i < num_cells; i++) {
    balancing_bytes[i] = balancing_status[i] ? 1 : 0;
  }
  
  uint16_t calculated_crc = calculate_crc16(balancing_bytes.data(), num_cells * sizeof(bool));
  return calculated_crc == expected_crc;
}

void BatteryMonitorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Battery Monitor...");
  this->last_data_time_ = millis();
}

void BatteryMonitorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Battery Monitor:");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  CRC Verification: Enabled for enhanced data integrity");
  ESP_LOGCONFIG(TAG, "  Supports both legacy and CRC-enabled data formats");
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
  DeserializationError error = deserializeJson(doc, json_data);
  
  if (error) {
    ESP_LOGW(TAG, "Failed to parse spec_data JSON: %s", error.c_str());
    ESP_LOGD(TAG, "  Raw data: %s", json_data.c_str());
    return;
  }
  
  // Handle new CRC-enabled format
  if (doc.containsKey("cell_data") && doc["cell_data"].is<JsonArray>() && 
      doc.containsKey("crc") && doc.containsKey("num_cells")) {
    
    JsonArray cell_data = doc["cell_data"];
    uint16_t received_crc = doc["crc"].as<uint16_t>();
    size_t num_cells = doc["num_cells"].as<size_t>();
    
    ESP_LOGD(TAG, "Received cell voltage data with CRC: %u, num_cells: %zu", received_crc, num_cells);
    
    if (cell_data.size() != num_cells) {
      ESP_LOGW(TAG, "Cell data size mismatch: expected %zu cells, got %zu", num_cells, cell_data.size());
    }
    
    // Verify CRC for data integrity
    if (verify_cell_voltage_crc(cell_data, received_crc, num_cells)) {
      ESP_LOGD(TAG, "Cell voltage CRC verification passed (CRC: %u)", received_crc);
      
      // Process verified cell data
      for (size_t i = 0; i < cell_data.size(); ++i) {
        if (cell_data[i].containsKey("cell") && cell_data[i].containsKey("voltage")) {
          int cell_number = cell_data[i]["cell"].as<int>() - 1; // Convert from 1-based to 0-based
          float voltage = cell_data[i]["voltage"].as<float>();
          
          auto it = this->cell_voltage_sensors_.find(cell_number);
          if (it != this->cell_voltage_sensors_.end() && it->second != nullptr) {
            // Only publish valid voltages (> 10mV = 0.01V)
            if (voltage > 0.01f) {
              it->second->publish_state(voltage);
              ESP_LOGVV(TAG, "Updated cell %d voltage: %.3fV", cell_number + 1, voltage);
            } else {
              ESP_LOGD(TAG, "Skipping invalid cell %d voltage: %.3fV (must be > 10mV)", cell_number + 1, voltage);
            }
          }
        }
      }
    } else {
      // Enhanced CRC error debugging
      ESP_LOGW(TAG, "Cell voltage CRC verification failed!");
      ESP_LOGW(TAG, "  Expected CRC: %u", received_crc);
      
      // Calculate what CRC we got for debugging
      std::vector<uint16_t> voltages_mv(num_cells);
      for (size_t i = 0; i < num_cells && i < cell_data.size(); i++) {
        if (cell_data[i].containsKey("voltage")) {
          float voltage_v = cell_data[i]["voltage"].as<float>();
          voltages_mv[i] = static_cast<uint16_t>(voltage_v * 1000.0f);
        }
      }
      uint16_t calculated_crc = calculate_crc16((const uint8_t*)voltages_mv.data(), num_cells * sizeof(uint16_t));
      ESP_LOGW(TAG, "  Calculated CRC: %u", calculated_crc);
      ESP_LOGW(TAG, "  Data may be corrupted or transmission error occurred");
      ESP_LOGD(TAG, "  Cell count: expected=%zu, received=%zu", num_cells, cell_data.size());
    }
  }
  // Handle legacy format for backwards compatibility
  else if (doc.containsKey("cell_voltages") && doc["cell_voltages"].is<JsonArray>()) {
    ESP_LOGD(TAG, "Processing legacy cell voltage format (no CRC)");
    JsonArray cell_voltages = doc["cell_voltages"];
    
    for (size_t i = 0; i < cell_voltages.size(); ++i) {
      auto it = this->cell_voltage_sensors_.find(i);
      if (it != this->cell_voltage_sensors_.end() && it->second != nullptr) {
        float voltage = cell_voltages[i].as<float>();
        
        // Only publish valid voltages (> 10mV = 0.01V)
        // Reject 0V and very low voltages as invalid readings
        if (voltage > 0.01f) {
          it->second->publish_state(voltage);
        } else {
          ESP_LOGD(TAG, "Skipping invalid cell %zu voltage: %.3fV (must be > 10mV)", i + 1, voltage);
        }
      }
    }
  } else {
    ESP_LOGW(TAG, "Unrecognized spec_data format");
    ESP_LOGD(TAG, "  Raw JSON: %s", json_data.c_str());
    ESP_LOGD(TAG, "  Expected either 'cell_data' (new format) or 'cell_voltages' (legacy format)");
  }
}

void BatteryMonitorComponent::handle_balancing_data(const std::string &json_data) {
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, json_data);
  
  if (error) {
    ESP_LOGW(TAG, "Failed to parse balancing_data JSON: %s", error.c_str());
    ESP_LOGD(TAG, "  Raw data: %s", json_data.c_str());
    return;
  }
  
  // Handle new CRC-enabled format
  if (doc.containsKey("cell_balancing_data") && doc["cell_balancing_data"].is<JsonArray>() && 
      doc.containsKey("crc") && doc.containsKey("num_cells")) {
    
    JsonArray cell_data = doc["cell_balancing_data"];
    uint16_t received_crc = doc["crc"].as<uint16_t>();
    size_t num_cells = doc["num_cells"].as<size_t>();
    
    ESP_LOGD(TAG, "Received cell balancing data with CRC: %u, num_cells: %zu", received_crc, num_cells);
    
    if (cell_data.size() != num_cells) {
      ESP_LOGW(TAG, "Balancing data size mismatch: expected %zu cells, got %zu", num_cells, cell_data.size());
    }
    
    // Verify CRC for data integrity
    if (verify_cell_balancing_crc(cell_data, received_crc, num_cells)) {
      ESP_LOGD(TAG, "Cell balancing CRC verification passed (CRC: %u)", received_crc);
      
      // Count active balancing cells for verification
      size_t active_cells = 0;
      for (size_t i = 0; i < cell_data.size(); ++i) {
        if (cell_data[i].containsKey("cell") && cell_data[i].containsKey("balancing")) {
          int cell_number = cell_data[i]["cell"].as<int>();
          bool balancing = cell_data[i]["balancing"].as<bool>();
          
          if (balancing) {
            active_cells++;
          }
          
          ESP_LOGVV(TAG, "Cell %d balancing: %s", cell_number, balancing ? "active" : "inactive");
        }
      }
      
      ESP_LOGD(TAG, "Verified balancing data: %zu/%zu cells actively balancing", active_cells, num_cells);
      
    } else {
      // Enhanced CRC error debugging
      ESP_LOGW(TAG, "Cell balancing CRC verification failed!");
      ESP_LOGW(TAG, "  Expected CRC: %u", received_crc);
      
      // Calculate what CRC we got for debugging
      std::vector<uint8_t> balancing_bytes(num_cells);
      size_t active_cells = 0;
      for (size_t i = 0; i < num_cells && i < cell_data.size(); i++) {
        if (cell_data[i].containsKey("balancing")) {
          bool balancing = cell_data[i]["balancing"].as<bool>();
          balancing_bytes[i] = balancing ? 1 : 0;
          if (balancing) active_cells++;
        }
      }
      uint16_t calculated_crc = calculate_crc16(balancing_bytes.data(), num_cells * sizeof(bool));
      ESP_LOGW(TAG, "  Calculated CRC: %u", calculated_crc);
      ESP_LOGW(TAG, "  Data may be corrupted or transmission error occurred");
      ESP_LOGD(TAG, "  Cell count: expected=%zu, received=%zu", num_cells, cell_data.size());
      ESP_LOGD(TAG, "  Active balancing cells in corrupted data: %zu", active_cells);
    }
  }
  // Handle legacy format for backwards compatibility
  else if (doc.containsKey("cell_balancing") && doc["cell_balancing"].is<JsonArray>()) {
    ESP_LOGD(TAG, "Processing legacy cell balancing format (no CRC)");
    JsonArray cell_balancing = doc["cell_balancing"];
    
    size_t active_cells = 0;
    for (size_t i = 0; i < cell_balancing.size(); ++i) {
      bool balancing = cell_balancing[i].as<bool>();
      if (balancing) {
        active_cells++;
      }
    }
    
    ESP_LOGD(TAG, "Legacy balancing data: %zu/%zu cells actively balancing", active_cells, cell_balancing.size());
    
  } else {
    ESP_LOGW(TAG, "Unrecognized balancing_data format");
    ESP_LOGD(TAG, "  Raw JSON: %s", json_data.c_str());
    ESP_LOGD(TAG, "  Expected either 'cell_balancing_data' (new format) or 'cell_balancing' (legacy format)");
  }
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