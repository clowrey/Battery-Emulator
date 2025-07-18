#include "serial_api.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <list>
#include "../../../USER_SETTINGS.h"
#include "../../battery/BATTERIES.h"
#include "../../communication/contactorcontrol/comm_contactorcontrol.h"
#include "../../datalayer/datalayer.h"
#include "../../lib/bblanchon-ArduinoJson/ArduinoJson.h"
#include "../../devboard/utils/events.h"
#include "../../devboard/utils/timer.h"
#include "../../devboard/utils/types.h"
#include "../../devboard/safety/safety.h"

char serial_api_msg[SERIAL_API_MSG_BUFFER_SIZE];
MyTimer publish_serial_timer(5000);  // publish timer - same as MQTT (5 seconds)
MyTimer check_serial_timer(800);     // check timer - same as MQTT (800ms)
bool serial_api_initialized = false;

// CRC16 calculation for data integrity verification
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

// Calculate CRC for cell voltage array
static uint16_t calculate_cell_voltage_crc(const uint16_t* cell_voltages, size_t num_cells) {
  return calculate_crc16((const uint8_t*)cell_voltages, num_cells * sizeof(uint16_t));
}

// Calculate CRC for cell balancing array  
static uint16_t calculate_cell_balancing_crc(const bool* cell_balancing, size_t num_cells) {
  return calculate_crc16((const uint8_t*)cell_balancing, num_cells * sizeof(bool));
}

static bool publish_serial_common_info(void);
static bool publish_serial_cell_voltages(void);
static bool publish_serial_cell_balancing(void);
static bool publish_serial_events(void);

/** Publish global values via serial */
static void publish_serial_values(void) {

  if (serial_api_publish("status", "\"online\"") == false) {
    return;
  }

  if (publish_serial_events() == false) {
    return;
  }

  if (publish_serial_common_info() == false) {
    return;
  }

#ifdef MQTT_PUBLISH_CELL_VOLTAGES
  if (publish_serial_cell_voltages() == false) {
    return;
  }
#endif

#ifdef MQTT_PUBLISH_CELL_VOLTAGES
  if (publish_serial_cell_balancing() == false) {
    return;
  }
#endif
}

void set_serial_battery_attributes(JsonDocument& doc, const DATALAYER_BATTERY_TYPE& battery, const String& suffix,
                                   bool supports_charged) {
  doc["SOC" + suffix] = ((float)battery.status.reported_soc) / 100.0;
  doc["SOC_real" + suffix] = ((float)battery.status.real_soc) / 100.0;
  doc["state_of_health" + suffix] = ((float)battery.status.soh_pptt) / 100.0;
  doc["temperature_min" + suffix] = ((float)((int16_t)battery.status.temperature_min_dC)) / 10.0;
  doc["temperature_max" + suffix] = ((float)((int16_t)battery.status.temperature_max_dC)) / 10.0;
  doc["cpu_temp" + suffix] = datalayer.system.info.CPU_temperature;
  doc["stat_batt_power" + suffix] = ((float)((int32_t)battery.status.active_power_W));
  doc["battery_current" + suffix] = ((float)((int16_t)battery.status.current_dA)) / 10.0;
  doc["battery_voltage" + suffix] = ((float)battery.status.voltage_dV) / 10.0;
  if (battery.info.number_of_cells != 0u && battery.status.cell_voltages_mV[battery.info.number_of_cells - 1] != 0u) {
    doc["cell_max_voltage" + suffix] = ((float)battery.status.cell_max_voltage_mV) / 1000.0;
    doc["cell_min_voltage" + suffix] = ((float)battery.status.cell_min_voltage_mV) / 1000.0;
    doc["cell_voltage_delta" + suffix] =
        ((float)battery.status.cell_max_voltage_mV) - ((float)battery.status.cell_min_voltage_mV);
  }
  doc["total_capacity" + suffix] = ((float)battery.info.total_capacity_Wh);
  doc["remaining_capacity_real" + suffix] = ((float)battery.status.remaining_capacity_Wh);
  doc["remaining_capacity" + suffix] = ((float)battery.status.reported_remaining_capacity_Wh);
  doc["max_discharge_power" + suffix] = ((float)battery.status.max_discharge_power_W);
  doc["max_charge_power" + suffix] = ((float)battery.status.max_charge_power_W);

  if (supports_charged) {
    if (datalayer.battery.status.total_charged_battery_Wh != 0 &&
        datalayer.battery.status.total_discharged_battery_Wh != 0) {
      doc["charged_energy" + suffix] = ((float)datalayer.battery.status.total_charged_battery_Wh);
      doc["discharged_energy" + suffix] = ((float)datalayer.battery.status.total_discharged_battery_Wh);
    }
  }

  // Add balancing data
  uint16_t active_cells = 0;
  if (battery.info.number_of_cells != 0u) {
    for (size_t i = 0; i < battery.info.number_of_cells; ++i) {
      if (battery.status.cell_balancing_status[i]) {
        active_cells++;
      }
    }
  }
  doc["balancing_active_cells" + suffix] = active_cells;
}

static std::vector<EventData> order_serial_events;

static bool publish_serial_common_info(void) {
  static JsonDocument doc;
  
  doc["bms_status"] = getBMSStatus(datalayer.battery.status.bms_status);
  doc["pause_status"] = get_emulator_pause_status();

  //only publish these values if BMS is active and we are communication with the battery
  if (datalayer.battery.status.CAN_battery_still_alive && allowed_to_send_CAN && millis() > BOOTUP_TIME) {
    set_serial_battery_attributes(doc, datalayer.battery, "", battery->supports_charged_energy());
  }

  if (battery2) {
    //only publish these values if BMS is active and we are communication with the battery
    if (datalayer.battery2.status.CAN_battery_still_alive && allowed_to_send_CAN && millis() > BOOTUP_TIME) {
      set_serial_battery_attributes(doc, datalayer.battery2, "_2", battery2->supports_charged_energy());
    }
  }
  
  serializeJson(doc, serial_api_msg);
  if (serial_api_publish("info", serial_api_msg) == false) {
#ifdef DEBUG_LOG
    logging.println("Common info Serial API msg could not be sent");
#endif  // DEBUG_LOG
    return false;
  }
  doc.clear();
  return true;
}

static bool publish_serial_cell_voltages(void) {
  static JsonDocument doc;

  // If cell voltages have been populated...
  if (datalayer.battery.info.number_of_cells != 0u &&
      datalayer.battery.status.cell_voltages_mV[datalayer.battery.info.number_of_cells - 1] != 0u) {

    // Create cell data array with cell numbers and voltages
    JsonArray cell_data = doc["cell_data"].to<JsonArray>();
    for (size_t i = 0; i < datalayer.battery.info.number_of_cells; ++i) {
      JsonObject cell = cell_data.createNestedObject();
      cell["cell"] = i + 1;  // Cell numbers start from 1
      cell["voltage"] = ((float)datalayer.battery.status.cell_voltages_mV[i]) / 1000.0;
    }
    
    // Calculate and add CRC for data integrity verification
    uint16_t crc = calculate_cell_voltage_crc(datalayer.battery.status.cell_voltages_mV, 
                                              datalayer.battery.info.number_of_cells);
    doc["crc"] = crc;
    doc["num_cells"] = datalayer.battery.info.number_of_cells;

    serializeJson(doc, serial_api_msg, sizeof(serial_api_msg));

    if (!serial_api_publish("spec_data", serial_api_msg)) {
#ifdef DEBUG_LOG
      logging.println("Cell voltage Serial API msg could not be sent");
#endif  // DEBUG_LOG
      return false;
    }
    doc.clear();
  }

  if (battery2) {
    // If cell voltages have been populated...
    if (datalayer.battery2.info.number_of_cells != 0u &&
        datalayer.battery2.status.cell_voltages_mV[datalayer.battery2.info.number_of_cells - 1] != 0u) {

      // Create cell data array with cell numbers and voltages
      JsonArray cell_data = doc["cell_data"].to<JsonArray>();
      for (size_t i = 0; i < datalayer.battery2.info.number_of_cells; ++i) {
        JsonObject cell = cell_data.createNestedObject();
        cell["cell"] = i + 1;  // Cell numbers start from 1
        cell["voltage"] = ((float)datalayer.battery2.status.cell_voltages_mV[i]) / 1000.0;
      }
      
      // Calculate and add CRC for data integrity verification
      uint16_t crc = calculate_cell_voltage_crc(datalayer.battery2.status.cell_voltages_mV, 
                                                datalayer.battery2.info.number_of_cells);
      doc["crc"] = crc;
      doc["num_cells"] = datalayer.battery2.info.number_of_cells;

      serializeJson(doc, serial_api_msg, sizeof(serial_api_msg));

      if (!serial_api_publish("spec_data_2", serial_api_msg)) {
#ifdef DEBUG_LOG
        logging.println("Cell voltage Serial API msg could not be sent");
#endif  // DEBUG_LOG
        return false;
      }
      doc.clear();
    }
  }
  return true;
}

static bool publish_serial_cell_balancing(void) {
  static JsonDocument doc;

  // If cell balancing data is available...
  if (datalayer.battery.info.number_of_cells != 0u) {

    // Create cell balancing data array with cell numbers and status
    JsonArray cell_data = doc["cell_balancing_data"].to<JsonArray>();
    for (size_t i = 0; i < datalayer.battery.info.number_of_cells; ++i) {
      JsonObject cell = cell_data.createNestedObject();
      cell["cell"] = i + 1;  // Cell numbers start from 1
      cell["balancing"] = datalayer.battery.status.cell_balancing_status[i];
    }
    
    // Calculate and add CRC for data integrity verification
    uint16_t crc = calculate_cell_balancing_crc(datalayer.battery.status.cell_balancing_status, 
                                                datalayer.battery.info.number_of_cells);
    doc["crc"] = crc;
    doc["num_cells"] = datalayer.battery.info.number_of_cells;

    serializeJson(doc, serial_api_msg, sizeof(serial_api_msg));

    if (!serial_api_publish("balancing_data", serial_api_msg)) {
#ifdef DEBUG_LOG
      logging.println("Cell balancing Serial API msg could not be sent");
#endif  // DEBUG_LOG
      return false;
    }
    doc.clear();
  }

  // Handle second battery if available
  if (battery2) {
    if (datalayer.battery2.info.number_of_cells != 0u) {

      // Create cell balancing data array with cell numbers and status
      JsonArray cell_data = doc["cell_balancing_data"].to<JsonArray>();
      for (size_t i = 0; i < datalayer.battery2.info.number_of_cells; ++i) {
        JsonObject cell = cell_data.createNestedObject();
        cell["cell"] = i + 1;  // Cell numbers start from 1
        cell["balancing"] = datalayer.battery2.status.cell_balancing_status[i];
      }
      
      // Calculate and add CRC for data integrity verification
      uint16_t crc = calculate_cell_balancing_crc(datalayer.battery2.status.cell_balancing_status, 
                                                  datalayer.battery2.info.number_of_cells);
      doc["crc"] = crc;
      doc["num_cells"] = datalayer.battery2.info.number_of_cells;

      serializeJson(doc, serial_api_msg, sizeof(serial_api_msg));

      if (!serial_api_publish("balancing_data_2", serial_api_msg)) {
#ifdef DEBUG_LOG
        logging.println("Cell balancing Serial API msg could not be sent");
#endif  // DEBUG_LOG
        return false;
      }
      doc.clear();
    }
  }
  return true;
}

bool publish_serial_events() {
  static JsonDocument doc;
  
  const EVENTS_STRUCT_TYPE* event_pointer;

  //clear the vector
  order_serial_events.clear();
  // Collect all events
  for (int i = 0; i < EVENT_NOF_EVENTS; i++) {
    event_pointer = get_event_pointer((EVENTS_ENUM_TYPE)i);
    if (event_pointer->occurences > 0 && !event_pointer->MQTTpublished) {
      order_serial_events.push_back({static_cast<EVENTS_ENUM_TYPE>(i), event_pointer});
    }
  }
  // Sort events by timestamp
  std::sort(order_serial_events.begin(), order_serial_events.end(), compareEventsByTimestampAsc);

  for (const auto& event : order_serial_events) {

    EVENTS_ENUM_TYPE event_handle = event.event_handle;
    event_pointer = event.event_pointer;

    doc["event_type"] = String(get_event_enum_string(event_handle));
    doc["severity"] = String(get_event_level_string(event_handle));
    doc["count"] = String(event_pointer->occurences);
    doc["data"] = String(event_pointer->data);
    doc["message"] = String(get_event_message_string(event_handle));
    doc["millis"] = String(event_pointer->timestamp);

    serializeJson(doc, serial_api_msg);
    if (!serial_api_publish("events", serial_api_msg)) {
#ifdef DEBUG_LOG
      logging.println("Events Serial API msg could not be sent");
#endif  // DEBUG_LOG
      return false;
    } else {
      set_event_MQTTpublished(event_handle);  // Reuse MQTT flag for serial API
    }
    doc.clear();
    //clear the vector
    order_serial_events.clear();
  }
  return true;
}

void init_serial_api(void) {
#ifdef SERIAL_API_DEDICATED_SERIAL
  // Initialize dedicated serial port (Serial2)
  Serial2.begin(SERIAL_API_BAUDRATE, SERIAL_8N1, SERIAL_API_RX_PIN, SERIAL_API_TX_PIN);
#ifdef DEBUG_LOG
  logging.printf("Serial API dedicated port initialized on pins TX:%d RX:%d at %d baud\n", 
                 SERIAL_API_TX_PIN, SERIAL_API_RX_PIN, SERIAL_API_BAUDRATE);
#endif  // DEBUG_LOG
#endif  // SERIAL_API_DEDICATED_SERIAL

  // Mark as initialized
  serial_api_initialized = true;
  
#ifdef DEBUG_LOG
  logging.println("Serial API initialized");
#endif  // DEBUG_LOG
}

void serial_api_loop(void) {
  // Only attempt to publish if check timer is elapsed
  if (check_serial_timer.elapsed()) {

    if (serial_api_initialized == false) {
      init_serial_api();
      return;
    }

    if (publish_serial_timer.elapsed())  // Every 5s
    {
      publish_serial_values();
    }
  }
}

bool serial_api_publish(const char* data_type, const char* json_msg) {
  bool success = true;
  
  // Send data in format: SERIAL_API:DATA_TYPE:JSON_MESSAGE\n
  
#ifdef SERIAL_API_USB_SERIAL
  // Send to main USB serial port (Serial)
  Serial.print("SERIAL_API:");
  Serial.print(data_type);
  Serial.print(":");
  Serial.print(json_msg);
  Serial.println();
#endif  // SERIAL_API_USB_SERIAL

#ifdef SERIAL_API_DEDICATED_SERIAL
  // Send to dedicated serial port (Serial2)
  Serial2.print("SERIAL_API:");
  Serial2.print(data_type);
  Serial2.print(":");
  Serial2.print(json_msg);
  Serial2.println();
#endif  // SERIAL_API_DEDICATED_SERIAL

  return success;  // Serial always succeeds (no network dependency)
} 