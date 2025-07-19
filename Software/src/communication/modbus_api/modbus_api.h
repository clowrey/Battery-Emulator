/**
 * Modbus RTU API for the battery emulator using eModbus library
 * 
 * This module sends cell voltage data via Modbus RTU over serial interface
 * using the eModbus server library for robust protocol handling
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
 * - Call modbus_api_loop() in main loop (no periodic work needed)
 * 
 * Data format: Modbus RTU holding registers with direct access to datalayer values
 * - Function Code 03 (Read Holding Registers) for reading cell voltages and settings
 * - Function Code 06 (Write Single Register) for writing settings
 * - Each cell voltage is read directly from datalayer in millivolts
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
#include "../../lib/eModbus-eModbus/ModbusServerRTU.h"
#include "../../lib/eModbus-eModbus/ModbusTypeDefs.h"
#include "../../devboard/hal/hal.h"

// Cell voltage register mapping
#define MODBUS_CELL_VOLTAGE_START_REG 0
#define MODBUS_CELL_VOLTAGE_COUNT 108

// Settings register mapping
#define MODBUS_BALANCING_HYSTERESIS_REG 108
#define MODBUS_SETTINGS_COUNT 1

// Total register count
#define MODBUS_TOTAL_REGISTER_COUNT (MODBUS_CELL_VOLTAGE_COUNT + MODBUS_SETTINGS_COUNT)

// Function declarations
void init_modbus_api(void);
void modbus_api_loop(void);

// eModbus worker functions
ModbusMessage modbus_api_fc03(ModbusMessage request);  // Read Holding Registers
ModbusMessage modbus_api_fc06(ModbusMessage request);  // Write Single Register

#endif 