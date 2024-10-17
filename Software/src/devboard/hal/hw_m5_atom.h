#ifndef __HW_STARK_H__
#define __HW_STARK_H__

/*

GPIOs on M5 Atom Lite

5 Pin lower connector
* 3.3V
* GPIO 22
* GPIO 19
* GPIO 23
* GPIO 33

4 Pin lower connector
* GPIO 21
* GPIO 25
* 5V 
* GND

Back grove connector
* GND
* 5V
* GPIO 26
* GPIO 32

*/

// Board boot-up time
#define BOOTUP_TIME 5000  // Time in ms it takes before system is considered fully started up

// Core assignment
#define CORE_FUNCTION_CORE 1
#define MODBUS_CORE 0
#define WIFI_CORE 0

// RS485
// #define PIN_5V_EN 16     // Not needed, GPIO 16 has hardware pullup for PSRAM compatibility
// #define RS485_EN_PIN 17  // Not needed, GPIO 17 is used as SCK input of MCP2517
//#define RS485_TX_PIN 22
//#define RS485_RX_PIN 21
// #define RS485_SE_PIN 19  // Not needed, GPIO 19 is available as extra GPIO via pin header

// CAN settings
#define CAN_1_TYPE ESP32CAN

// CAN1 PIN mappings, do not change these unless you are adding on extra hardware to the PCB
#define CAN_TX_PIN GPIO_NUM_26
#define CAN_RX_PIN GPIO_NUM_32
// #define CAN_SE_PIN 23 // (No function, GPIO 23 used instead as MCP_SCK)

// CAN_FD defines
#define MCP2517_SCK 19  // SCK input  of MCP2517
#define MCP2517_SDI 33  // SDI input  of MCP2517
#define MCP2517_SDO 23  // SDO output of MCP2517
#define MCP2517_CS 22   // CS  input  of MCP2517
#define MCP2517_INT 21  // INT output of MCP2517

// Contactor handling
//#define POSITIVE_CONTACTOR_PIN 32
//#define NEGATIVE_CONTACTOR_PIN 33
//#define PRECHARGE_PIN 25
//#define BMS_POWER 23

// SMA CAN contactor pins
//#define INVERTER_CONTACTOR_ENABLE_PIN 19

// LED
#define LED_PIN 27
#define LED_MAX_BRIGHTNESS 40

// Equipment stop pin
//#define EQUIPMENT_STOP_PIN 2

/* ----- Error checks below, don't change (can't be moved to separate file) ----- */
#ifndef HW_CONFIGURED
#define HW_CONFIGURED
#else
#error Multiple HW defined! Please select a single HW
#endif

#endif
