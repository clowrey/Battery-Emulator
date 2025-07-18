/**
 * Serial API for the battery emulator
 * 
 * This module sends the same data as MQTT via serial interface
 * at the same update intervals (5 second main data, 800ms check)
 * 
 * Usage:
 * - Initialize with init_serial_api()
 * - Call serial_api_loop() in main loop
 * 
 * Data format: JSON over serial at 115200 baud
 */

#ifndef __SERIAL_API_H__
#define __SERIAL_API_H__

#include <Arduino.h>
#include "../../include.h"

#define SERIAL_API_MSG_BUFFER_SIZE (1024)

extern char serial_api_msg[SERIAL_API_MSG_BUFFER_SIZE];

void init_serial_api(void);
void serial_api_loop(void);
bool serial_api_publish(const char* data_type, const char* json_msg);

#endif 