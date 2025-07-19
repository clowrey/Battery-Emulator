#include "modbus_api.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include "../../../USER_SETTINGS.h"
#include "../../datalayer/datalayer.h"
#include "../../devboard/utils/timer.h"
#include "../../devboard/utils/logging.h"
#include "../nvm/comm_nvm.h"

#ifdef MODBUS_API

// eModbus server instance
static ModbusServerRTU modbus_server(2000);  // 2000ms timeout

// State variables
static bool modbus_api_initialized = false;

// Worker function for FC03 - Read Holding Registers
ModbusMessage modbus_api_fc03(ModbusMessage request) {
  ModbusMessage response;
  uint16_t start_address = 0;
  uint16_t register_count = 0;
  
  // Extract parameters from request
  request.get(2, start_address);
  request.get(4, register_count);
  
#ifdef DEBUG_LOG
  logging.printf("Modbus API FC03: Read %d registers starting at %d\n", register_count, start_address);
#endif
  
  // Validate register range
  if (start_address >= MODBUS_TOTAL_REGISTER_COUNT || 
      register_count == 0 || 
      (start_address + register_count) > MODBUS_TOTAL_REGISTER_COUNT ||
      register_count > 125) {  // Max registers per response (limited by frame size)
    
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  
  // Build successful response
  uint8_t byte_count = register_count * 2;
  response.add(request.getServerID(), request.getFunctionCode(), byte_count);
  
  // Add register data directly from datalayer (big-endian format)
  for (uint16_t i = 0; i < register_count; i++) {
    uint16_t reg_addr = start_address + i;
    uint16_t value;
    
    if (reg_addr < MODBUS_CELL_VOLTAGE_COUNT) {
      // Read cell voltage directly from datalayer
      if (reg_addr < datalayer.battery.info.number_of_cells) {
        value = datalayer.battery.status.cell_voltages_mV[reg_addr];
      } else {
        value = 0;  // Zero for non-existent cells
      }
    } else if (reg_addr == MODBUS_BALANCING_HYSTERESIS_REG) {
      // Read setting directly from datalayer
      value = datalayer.battery.settings.balancing_hysteresis_mV;
    } else {
      // Should not reach here due to validation above, but safety fallback
      value = 0;
    }
    
    response.add(value);
  }
  
  return response;
}

// Worker function for FC06 - Write Single Register
ModbusMessage modbus_api_fc06(ModbusMessage request) {
  ModbusMessage response;
  uint16_t register_address = 0;
  uint16_t register_value = 0;
  
  // Extract parameters from request
  request.get(2, register_address);
  request.get(4, register_value);
  
#ifdef DEBUG_LOG
  logging.printf("Modbus API FC06: Write register %d with value %d\n", register_address, register_value);
#endif
  
  // Validate register address (only settings registers are writable)
  if (register_address < MODBUS_CELL_VOLTAGE_COUNT) {
    // Cell voltage registers are read-only
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  
  if (register_address >= MODBUS_TOTAL_REGISTER_COUNT) {
    // Invalid register address
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  
  // Handle specific settings registers
  if (register_address == MODBUS_BALANCING_HYSTERESIS_REG) {
    // Validate balancing hysteresis range (1-100 mV)
    if (register_value < 1 || register_value > 100) {
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
      
#ifdef DEBUG_LOG
      logging.printf("Modbus API: Invalid balancing hysteresis value: %d (valid range: 1-100 mV)\n", register_value);
#endif
      return response;
    }
    
    // Update datalayer directly
    datalayer.battery.settings.balancing_hysteresis_mV = register_value;
    
    // Store settings to NVM
    store_settings();
    
#ifdef DEBUG_LOG
    logging.printf("Modbus API: Updated balancing hysteresis to %d mV\n", register_value);
#endif
  }
  
  // Send successful response (echo the request)
  response.add(request.getServerID(), request.getFunctionCode(), register_address, register_value);
  
  return response;
}

// Initialize Modbus API
void init_modbus_api(void) {
#ifdef DEBUG_LOG
  logging.println("Initializing Modbus API with eModbus...");
#endif

  // Prepare hardware serial
  RTUutils::prepareHardwareSerial(Serial2);
  
  // Initialize serial port for Modbus RTU communication
  Serial2.begin(MODBUS_API_BAUDRATE, SERIAL_8N1, MODBUS_API_RX_PIN, MODBUS_API_TX_PIN);
  
  // Register worker functions for supported function codes
  modbus_server.registerWorker(MODBUS_API_SLAVE_ID, READ_HOLD_REGISTER, &modbus_api_fc03);
  modbus_server.registerWorker(MODBUS_API_SLAVE_ID, WRITE_HOLD_REGISTER, &modbus_api_fc06);
  
  // Start ModbusRTU background task
  modbus_server.begin(Serial2, MODBUS_CORE);
  
  modbus_api_initialized = true;
  
#ifdef DEBUG_LOG
  logging.printf("Modbus API initialized on pins TX:%d RX:%d at %d baud, Slave ID: %d\n", 
                 MODBUS_API_TX_PIN, MODBUS_API_RX_PIN, MODBUS_API_BAUDRATE, MODBUS_API_SLAVE_ID);
#endif
}

// Main Modbus API loop function - simplified since no periodic updates needed
void modbus_api_loop(void) {
  if (!modbus_api_initialized) {
    init_modbus_api();
  }
  // eModbus handles all communication in background task, no periodic work needed
}

#endif  // MODBUS_API 