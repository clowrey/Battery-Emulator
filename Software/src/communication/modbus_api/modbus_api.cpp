#include "modbus_api.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include "../../../USER_SETTINGS.h"
#include "../../datalayer/datalayer.h"
#include "../../devboard/utils/timer.h"
#include "../../devboard/utils/logging.h"
#include "../nvm/comm_nvm.h"

#ifdef MODBUS_API

// Modbus RTU data storage
static uint16_t modbus_cell_voltage_registers[MODBUS_CELL_VOLTAGE_COUNT];
static uint16_t modbus_settings_registers[MODBUS_SETTINGS_COUNT];
static uint8_t modbus_rx_buffer[MODBUS_API_RX_BUFFER_SIZE];
static uint8_t modbus_tx_buffer[MODBUS_API_TX_BUFFER_SIZE];
static uint16_t modbus_rx_index = 0;
static bool modbus_api_initialized = false;
static unsigned long last_frame_time = 0;
static unsigned long last_update_time = 0;

// Initialize Modbus API
void init_modbus_api(void) {
#ifdef DEBUG_LOG
  logging.println("Initializing Modbus API...");
#endif

  // Initialize serial port for Modbus RTU communication
  Serial2.begin(MODBUS_API_BAUDRATE, SERIAL_8N1, MODBUS_API_RX_PIN, MODBUS_API_TX_PIN);
  
  // Initialize cell voltage registers to zero
  for (int i = 0; i < MODBUS_CELL_VOLTAGE_COUNT; i++) {
    modbus_cell_voltage_registers[i] = 0;
  }
  
  // Initialize settings registers from datalayer
  update_settings_registers();
  
  // Reset buffers and indices
  modbus_rx_index = 0;
  last_frame_time = millis();
  last_update_time = millis();
  
  modbus_api_initialized = true;
  
#ifdef DEBUG_LOG
  logging.printf("Modbus API initialized on pins TX:%d RX:%d at %d baud, Slave ID: %d\n", 
                 MODBUS_API_TX_PIN, MODBUS_API_RX_PIN, MODBUS_API_BAUDRATE, MODBUS_API_SLAVE_ID);
#endif
}

// Main Modbus API loop function
void modbus_api_loop(void) {
  if (!modbus_api_initialized) {
    init_modbus_api();
    return;
  }
  
  unsigned long current_time = millis();
  
  // Update registers periodically
  if (current_time - last_update_time >= MODBUS_API_UPDATE_INTERVAL_MS) {
    update_cell_voltage_registers();
    update_settings_registers();
    last_update_time = current_time;
  }
  
  // Process incoming Modbus requests
  while (Serial2.available()) {
    uint8_t received_byte = Serial2.read();
    
    // Check for frame timeout (inter-frame delay)
    if (current_time - last_frame_time > MODBUS_RTU_INTER_FRAME_DELAY_MS) {
      modbus_rx_index = 0;  // Reset buffer if too much time has passed
    }
    
    // Store received byte
    if (modbus_rx_index < MODBUS_API_RX_BUFFER_SIZE) {
      modbus_rx_buffer[modbus_rx_index++] = received_byte;
    } else {
      // Buffer overflow, reset
      modbus_rx_index = 0;
      modbus_rx_buffer[modbus_rx_index++] = received_byte;
    }
    
    last_frame_time = current_time;
  }
  
  // Check if we have a complete frame after inter-frame delay
  if (modbus_rx_index > 0 && (current_time - last_frame_time) > MODBUS_RTU_INTER_FRAME_DELAY_MS) {
    process_modbus_request(modbus_rx_buffer, modbus_rx_index);
    modbus_rx_index = 0;  // Reset for next frame
  }
}

// Update cell voltage registers from datalayer
void update_cell_voltage_registers(void) {
  // Copy cell voltages from datalayer to Modbus holding registers
  uint8_t num_cells = datalayer.battery.info.number_of_cells;
  
  // Ensure we don't exceed our register array bounds
  if (num_cells > MODBUS_CELL_VOLTAGE_COUNT) {
    num_cells = MODBUS_CELL_VOLTAGE_COUNT;
  }
  
  // Copy actual cell voltages
  for (int i = 0; i < num_cells; i++) {
    modbus_cell_voltage_registers[i] = datalayer.battery.status.cell_voltages_mV[i];
  }
  
  // Fill remaining registers with zero if we have fewer than 108 cells
  for (int i = num_cells; i < MODBUS_CELL_VOLTAGE_COUNT; i++) {
    modbus_cell_voltage_registers[i] = 0;
  }
  
#ifdef DEBUG_LOG
  static unsigned long last_debug_time = 0;
  unsigned long current_time = millis();
  
  // Debug output every 10 seconds
  if (current_time - last_debug_time > 10000) {
    logging.printf("Modbus API: Updated %d cell voltages. First 5 cells: %dmV, %dmV, %dmV, %dmV, %dmV\n",
                   num_cells,
                   modbus_cell_voltage_registers[0],
                   modbus_cell_voltage_registers[1], 
                   modbus_cell_voltage_registers[2],
                   modbus_cell_voltage_registers[3],
                   modbus_cell_voltage_registers[4]);
    last_debug_time = current_time;
  }
#endif
}

// Update settings registers from datalayer
void update_settings_registers(void) {
  // Update balancing hysteresis register
  modbus_settings_registers[0] = datalayer.battery.settings.balancing_hysteresis_mV;
  
#ifdef DEBUG_LOG
  static unsigned long last_debug_time = 0;
  unsigned long current_time = millis();
  
  // Debug output every 10 seconds
  if (current_time - last_debug_time > 10000) {
    logging.printf("Modbus API: Updated settings registers. Balancing hysteresis: %dmV\n",
                   modbus_settings_registers[0]);
    last_debug_time = current_time;
  }
#endif
}

// Calculate Modbus RTU CRC16
uint16_t calculate_modbus_crc(uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

// Process incoming Modbus request
void process_modbus_request(uint8_t* request, uint16_t request_length) {
  // Minimum valid Modbus request: Address(1) + Function(1) + Data(4) + CRC(2) = 8 bytes
  if (request_length < 8) {
    return;  // Invalid frame length
  }
  
  uint8_t slave_address = request[0];
  uint8_t function_code = request[1];
  
  // Check if this request is for our slave address or broadcast
  if (slave_address != MODBUS_API_SLAVE_ID && slave_address != 0) {
    return;  // Not for us
  }
  
  // Verify CRC
  uint16_t received_crc = (request[request_length - 1] << 8) | request[request_length - 2];
  uint16_t calculated_crc = calculate_modbus_crc(request, request_length - 2);
  
  if (received_crc != calculated_crc) {
#ifdef DEBUG_LOG
    logging.printf("Modbus API: CRC error. Received: 0x%04X, Calculated: 0x%04X\n", received_crc, calculated_crc);
#endif
    return;  // CRC mismatch
  }
  
  // Process function code
  switch (function_code) {
    case MODBUS_FUNC_READ_HOLDING_REGISTERS: {
      // Parse read holding registers request
      uint16_t start_address = (request[2] << 8) | request[3];
      uint16_t register_count = (request[4] << 8) | request[5];
      
      // Validate register range
      if (start_address >= MODBUS_TOTAL_REGISTER_COUNT || 
          register_count == 0 || 
          (start_address + register_count) > MODBUS_TOTAL_REGISTER_COUNT ||
          register_count > 125) {  // Max registers per response (limited by frame size)
        
        // Send exception response - Illegal Data Address
        uint8_t exception_response[] = {
          slave_address,
          (uint8_t)(function_code | 0x80),  // Error flag
          0x02,  // Exception code: Illegal Data Address
          0x00, 0x00  // CRC placeholder
        };
        
        uint16_t crc = calculate_modbus_crc(exception_response, 3);
        exception_response[3] = crc & 0xFF;
        exception_response[4] = (crc >> 8) & 0xFF;
        
        send_modbus_response(exception_response, 5);
        return;
      }
      
      // Build successful response
      uint8_t byte_count = register_count * 2;
      uint16_t response_length = 3 + byte_count + 2;  // Address + Function + ByteCount + Data + CRC
      
      modbus_tx_buffer[0] = slave_address;
      modbus_tx_buffer[1] = function_code;
      modbus_tx_buffer[2] = byte_count;
      
      // Copy register data (big-endian format)
      for (uint16_t i = 0; i < register_count; i++) {
        uint16_t current_address = start_address + i;
        uint16_t reg_value;
        
        // Determine which register array to read from
        if (current_address < MODBUS_CELL_VOLTAGE_COUNT) {
          // Cell voltage register
          reg_value = modbus_cell_voltage_registers[current_address];
        } else {
          // Settings register
          reg_value = modbus_settings_registers[current_address - MODBUS_CELL_VOLTAGE_COUNT];
        }
        
        modbus_tx_buffer[3 + (i * 2)] = (reg_value >> 8) & 0xFF;      // High byte
        modbus_tx_buffer[3 + (i * 2) + 1] = reg_value & 0xFF;         // Low byte
      }
      
      // Calculate and append CRC
      uint16_t crc = calculate_modbus_crc(modbus_tx_buffer, response_length - 2);
      modbus_tx_buffer[response_length - 2] = crc & 0xFF;
      modbus_tx_buffer[response_length - 1] = (crc >> 8) & 0xFF;
      
      send_modbus_response(modbus_tx_buffer, response_length);
      
#ifdef DEBUG_LOG
      logging.printf("Modbus API: Read %d registers starting at %d\n", register_count, start_address);
#endif
      break;
    }
    
    case MODBUS_FUNC_WRITE_SINGLE_REGISTER: {
      process_write_single_register(request, request_length);
      break;
    }
    
    default:
      // Send exception response - Illegal Function
      uint8_t exception_response[] = {
        slave_address,
        (uint8_t)(function_code | 0x80),  // Error flag
        0x01,  // Exception code: Illegal Function
        0x00, 0x00  // CRC placeholder
      };
      
      uint16_t crc = calculate_modbus_crc(exception_response, 3);
      exception_response[3] = crc & 0xFF;
      exception_response[4] = (crc >> 8) & 0xFF;
      
      send_modbus_response(exception_response, 5);
      
#ifdef DEBUG_LOG
      logging.printf("Modbus API: Unsupported function code: 0x%02X\n", function_code);
#endif
      break;
  }
}

// Process write single register request
void process_write_single_register(uint8_t* request, uint16_t request_length) {
  // Write single register request: Address(1) + Function(1) + Address(2) + Value(2) + CRC(2) = 8 bytes
  if (request_length != 8) {
    return;  // Invalid frame length
  }
  
  uint8_t slave_address = request[0];
  uint16_t register_address = (request[2] << 8) | request[3];
  uint16_t register_value = (request[4] << 8) | request[5];
  
  // Validate register address (only settings registers are writable)
  if (register_address < MODBUS_CELL_VOLTAGE_COUNT) {
    // Cell voltage registers are read-only
    uint8_t exception_response[] = {
      slave_address,
      (uint8_t)(MODBUS_FUNC_WRITE_SINGLE_REGISTER | 0x80),  // Error flag
      0x02,  // Exception code: Illegal Data Address
      0x00, 0x00  // CRC placeholder
    };
    
    uint16_t crc = calculate_modbus_crc(exception_response, 3);
    exception_response[3] = crc & 0xFF;
    exception_response[4] = (crc >> 8) & 0xFF;
    
    send_modbus_response(exception_response, 5);
    return;
  }
  
  if (register_address >= MODBUS_TOTAL_REGISTER_COUNT) {
    // Invalid register address
    uint8_t exception_response[] = {
      slave_address,
      (uint8_t)(MODBUS_FUNC_WRITE_SINGLE_REGISTER | 0x80),  // Error flag
      0x02,  // Exception code: Illegal Data Address
      0x00, 0x00  // CRC placeholder
    };
    
    uint16_t crc = calculate_modbus_crc(exception_response, 3);
    exception_response[3] = crc & 0xFF;
    exception_response[4] = (crc >> 8) & 0xFF;
    
    send_modbus_response(exception_response, 5);
    return;
  }
  
  // Handle specific settings registers
  if (register_address == MODBUS_BALANCING_HYSTERESIS_REG) {
    // Validate balancing hysteresis range (1-100 mV)
    if (register_value < 1 || register_value > 100) {
      uint8_t exception_response[] = {
        slave_address,
        (uint8_t)(MODBUS_FUNC_WRITE_SINGLE_REGISTER | 0x80),  // Error flag
        0x03,  // Exception code: Illegal Data Value
        0x00, 0x00  // CRC placeholder
      };
      
      uint16_t crc = calculate_modbus_crc(exception_response, 3);
      exception_response[3] = crc & 0xFF;
      exception_response[4] = (crc >> 8) & 0xFF;
      
      send_modbus_response(exception_response, 5);
      
#ifdef DEBUG_LOG
      logging.printf("Modbus API: Invalid balancing hysteresis value: %d (valid range: 1-100 mV)\n", register_value);
#endif
      return;
    }
    
    // Update datalayer and local register
    datalayer.battery.settings.balancing_hysteresis_mV = register_value;
    modbus_settings_registers[register_address - MODBUS_CELL_VOLTAGE_COUNT] = register_value;
    
    // Store settings to NVM
    store_settings();
    
#ifdef DEBUG_LOG
    logging.printf("Modbus API: Updated balancing hysteresis to %d mV\n", register_value);
#endif
  }
  
  // Send successful response (echo the request)
  uint8_t response[] = {
    slave_address,
    MODBUS_FUNC_WRITE_SINGLE_REGISTER,
    (uint8_t)((register_address >> 8) & 0xFF),  // Register address high byte
    (uint8_t)(register_address & 0xFF),         // Register address low byte
    (uint8_t)((register_value >> 8) & 0xFF),    // Register value high byte
    (uint8_t)(register_value & 0xFF),           // Register value low byte
    0x00, 0x00  // CRC placeholder
  };
  
  uint16_t crc = calculate_modbus_crc(response, 6);
  response[6] = crc & 0xFF;
  response[7] = (crc >> 8) & 0xFF;
  
  send_modbus_response(response, 8);
}

// Send Modbus response
void send_modbus_response(uint8_t* response, uint16_t response_length) {
  // Add small delay before transmission to ensure line is ready
  delay(1);
  
  // Send response
  Serial2.write(response, response_length);
  Serial2.flush();  // Wait for transmission to complete
  
#ifdef DEBUG_LOG
  logging.printf("Modbus API: Sent response, %d bytes\n", response_length);
#endif
}

#endif  // MODBUS_API 