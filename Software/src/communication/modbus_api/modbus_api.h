/**
 * Modbus RTU API for the battery emulator
 * 
 * This module sends cell voltage data via Modbus RTU over serial interface
 * at regular intervals, optimized for data efficiency by sending binary data
 * 
 * Configuration options in USER_SETTINGS.h:
 * - MODBUS_API: Enable/disable the entire Modbus API module
 * - MODBUS_API_TX_PIN: TX pin for Modbus RTU serial port
 * - MODBUS_API_RX_PIN: RX pin for Modbus RTU serial port
 * - MODBUS_API_BAUDRATE: Baud rate for Modbus RTU serial port
 * - MODBUS_API_SLAVE_ID: Modbus slave ID for the device
 * 
 * Usage:
 * - Initialize with init_modbus_api()
 * - Call modbus_api_loop() in main loop
 * 
 * Data format: Modbus RTU holding registers with binary cell voltage data and settings
 * - Function Code 03 (Read Holding Registers) for reading cell voltages and settings
 * - Function Code 06 (Write Single Register) for writing settings
 * - Each cell voltage is stored as 16-bit value in millivolts
 * - Total of 109 registers: 108 cell voltages + 1 settings register
 * - Register mapping: 
 *   - Register 0-107 = Cell voltages 1-108 in mV (Read Only)
 *   - Register 108 = Balancing hysteresis in mV (Read/Write, range 1-100)
 */

#ifndef __MODBUS_API_H__
#define __MODBUS_API_H__

#include <Arduino.h>
#include "../../include.h"
#include "../../../USER_SETTINGS.h"

// Modbus function codes
#define MODBUS_FUNC_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER 0x06

// Modbus RTU specific constants
#define MODBUS_RTU_MAX_FRAME_SIZE 256
#define MODBUS_RTU_CRC_SIZE 2
#define MODBUS_RTU_ADDR_SIZE 1
#define MODBUS_RTU_FUNC_SIZE 1

// Cell voltage register mapping
#define MODBUS_CELL_VOLTAGE_START_REG 0
#define MODBUS_CELL_VOLTAGE_COUNT 108

// Settings register mapping
#define MODBUS_BALANCING_HYSTERESIS_REG 108
#define MODBUS_SETTINGS_COUNT 1

// Total register count
#define MODBUS_TOTAL_REGISTER_COUNT (MODBUS_CELL_VOLTAGE_COUNT + MODBUS_SETTINGS_COUNT)

// Buffer sizes
#define MODBUS_API_RX_BUFFER_SIZE 256
#define MODBUS_API_TX_BUFFER_SIZE 256

// Timing constants
#define MODBUS_RTU_INTER_FRAME_DELAY_MS 5   // Minimum delay between frames
#define MODBUS_API_UPDATE_INTERVAL_MS 1000  // Update cell voltages every 1 second

void init_modbus_api(void);
void modbus_api_loop(void);

// Modbus RTU specific functions
uint16_t calculate_modbus_crc(uint8_t* data, uint16_t length);
void process_modbus_request(uint8_t* request, uint16_t request_length);
void send_modbus_response(uint8_t* response, uint16_t response_length);
void update_cell_voltage_registers(void);
void update_settings_registers(void);
void process_write_single_register(uint8_t* request, uint16_t request_length);

#endif 