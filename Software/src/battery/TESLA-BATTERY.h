#ifndef TESLA_BATTERY_H
#define TESLA_BATTERY_H
#include "../include.h"

#define BATTERY_SELECTED

/* Modify these if needed */
#define LFP_CHEMISTRY // Enable this line to startup in LFP mode
#define MAXCHARGEPOWERALLOWED 15000     // 15000W we use a define since the value supplied by Tesla is always 0
#define MAXDISCHARGEPOWERALLOWED 60000  // 60000W we use a define since the value supplied by Tesla is always 0

/* Do not change the defines below */
#define RAMPDOWN_SOC 800            // 80.0 SOC% to start ramping down from max charge power towards 0 at 100.00%
#define RAMPDOWNPOWERALLOWED 10000  // What power we ramp down from towards top balancing
#define FLOAT_MAX_POWER_W 400       // W, what power to allow for top balancing battery
#define FLOAT_START_MV 150           // mV, how many mV under overvoltage to start float charging

#define MAX_PACK_VOLTAGE_SX_NCMA 4600   // V+1, if pack voltage goes over this, charge stops
#define MIN_PACK_VOLTAGE_SX_NCMA 3100   // V+1, if pack voltage goes over this, charge stops
#define MAX_PACK_VOLTAGE_3Y_NCMA 4030   // V+1, if pack voltage goes over this, charge stops
#define MIN_PACK_VOLTAGE_3Y_NCMA 3100   // V+1, if pack voltage goes below this, discharge stops
#define MAX_PACK_VOLTAGE_3Y_LFP 3740    // V+1, Voltage sent to inverter as target float voltage
#define MIN_PACK_VOLTAGE_3Y_LFP 3240    // = 3.00V per cell if pack voltage goes below this, discharge stops
#define MAX_CELL_DEVIATION_NCA_NCM 500  //LED turns yellow on the board if mv delta exceeds this value
#define MAX_CELL_DEVIATION_LFP 400      //LED turns yellow on the board if mv delta exceeds this value
#define MAX_CELL_VOLTAGE_NCA_NCM 4250   //Battery set to float watts if one cell goes over this value
#define MIN_CELL_VOLTAGE_NCA_NCM 2950   //Battery is put into emergency stop if one cell goes below this value
#define MAX_CELL_VOLTAGE_LFP 3650       //Battery set to float watts if one cell goes over this value
#define MIN_CELL_VOLTAGE_LFP 2968       //Battery is put into emergency stop if one cell goes below this value

void printFaultCodesIfActive();
void printDebugIfActive(uint8_t symbol, const char* message);
void print_int_with_units(char* header, int value, char* units);
void print_SOC(char* header, int SOC);
void setup_battery(void);
void transmit_can(CAN_frame* tx_frame, int interface);
#ifdef DOUBLE_BATTERY
void printFaultCodesIfActive_battery2();
#endif  //DOUBLE_BATTERY

#endif
