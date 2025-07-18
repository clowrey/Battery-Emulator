/**
 * Serial API for the battery emulator
 * 
 * This module sends the same data as MQTT via serial interface
 * at the same update intervals (5 second main data, 800ms check)
 * 
 * Configuration options in USER_SETTINGS.h:
 * - SERIAL_API: Enable/disable the entire Serial API module
 * - SERIAL_API_USB_SERIAL: Enable output on main USB serial port (Serial)
 * - SERIAL_API_DEDICATED_SERIAL: Enable output on dedicated serial port (Serial2)
 * - SERIAL_API_TX_PIN: TX pin for dedicated serial port
 * - SERIAL_API_RX_PIN: RX pin for dedicated serial port
 * - SERIAL_API_BAUDRATE: Baud rate for dedicated serial port
 * 
 * Usage:
 * - Initialize with init_serial_api()
 * - Call serial_api_loop() in main loop
 * 
 * Data format: JSON over serial with CRC verification
 * Format: SERIAL_API:<data_type>:<json_message>
 * 
 * Cell data format with error checking:
 * - Cell voltages: {"cell_data": [{"cell": 1, "voltage": 3.45}, ...], "crc": 12345, "num_cells": 96}
 * - Cell balancing: {"cell_balancing_data": [{"cell": 1, "balancing": true}, ...], "crc": 12345, "num_cells": 96}
 * 
 * CRC verification:
 * - Uses CRC-CCITT (polynomial 0x1021) for data integrity
 * - CRC calculated over raw cell data arrays before JSON conversion
 * - Recipients should verify CRC matches expected value for data validation
 */

#ifndef __SERIAL_API_H__
#define __SERIAL_API_H__

#include <Arduino.h>
#include "../../include.h"
#include "../../../USER_SETTINGS.h"

#define SERIAL_API_MSG_BUFFER_SIZE (1024)

extern char serial_api_msg[SERIAL_API_MSG_BUFFER_SIZE];

void init_serial_api(void);
void serial_api_loop(void);
bool serial_api_publish(const char* data_type, const char* json_msg);

#endif 