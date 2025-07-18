#include "BATMAN-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../include.h"

#ifdef BATMAN_BATTERY
#include <cmath>  // For isnan

// CRC table for Batman data
#define crcPolyBmData 0x025b

static const uint16_t crc14table[256] =
{
    0x0000, 0x025b, 0x04b6, 0x06ed, 0x096c, 0x0b37, 0x0dda, 0x0f81,
    0x12d8, 0x1083, 0x166e, 0x1435, 0x1bb4, 0x19ef, 0x1f02, 0x1d59,
    0x25b0, 0x27eb, 0x2106, 0x235d, 0x2cdc, 0x2e87, 0x286a, 0x2a31,
    0x3768, 0x3533, 0x33de, 0x3185, 0x3e04, 0x3c5f, 0x3ab2, 0x38e9,
    0x093b, 0x0b60, 0x0d8d, 0x0fd6, 0x0057, 0x020c, 0x04e1, 0x06ba,
    0x1be3, 0x19b8, 0x1f55, 0x1d0e, 0x128f, 0x10d4, 0x1639, 0x1462,
    0x2c8b, 0x2ed0, 0x283d, 0x2a66, 0x25e7, 0x27bc, 0x2151, 0x230a,
    0x3e53, 0x3c08, 0x3ae5, 0x38be, 0x373f, 0x3564, 0x3389, 0x31d2,
    0x1276, 0x102d, 0x16c0, 0x149b, 0x1b1a, 0x1941, 0x1fac, 0x1df7,
    0x00ae, 0x02f5, 0x0418, 0x0643, 0x09c2, 0x0b99, 0x0d74, 0x0f2f,
    0x37c6, 0x359d, 0x3370, 0x312b, 0x3eaa, 0x3cf1, 0x3a1c, 0x3847,
    0x251e, 0x2745, 0x21a8, 0x23f3, 0x2c72, 0x2e29, 0x28c4, 0x2a9f,
    0x1b4d, 0x1916, 0x1ffb, 0x1da0, 0x1221, 0x107a, 0x1697, 0x14cc,
    0x0995, 0x0bce, 0x0d23, 0x0f78, 0x00f9, 0x02a2, 0x044f, 0x0614,
    0x3efd, 0x3ca6, 0x3a4b, 0x3810, 0x3791, 0x35ca, 0x3327, 0x317c,
    0x2c25, 0x2e7e, 0x2893, 0x2ac8, 0x2549, 0x2712, 0x21ff, 0x23a4,
    0x24ec, 0x26b7, 0x205a, 0x2201, 0x2d80, 0x2fdb, 0x2936, 0x2b6d,
    0x3634, 0x346f, 0x3282, 0x30d9, 0x3f58, 0x3d03, 0x3bee, 0x39b5,
    0x015c, 0x0307, 0x05ea, 0x07b1, 0x0830, 0x0a6b, 0x0c86, 0x0edd,
    0x1384, 0x11df, 0x1732, 0x1569, 0x1ae8, 0x18b3, 0x1e5e, 0x1c05,
    0x2dd7, 0x2f8c, 0x2961, 0x2b3a, 0x24bb, 0x26e0, 0x200d, 0x2256,
    0x3f0f, 0x3d54, 0x3bb9, 0x39e2, 0x3663, 0x3438, 0x32d5, 0x308e,
    0x0867, 0x0a3c, 0x0cd1, 0x0e8a, 0x010b, 0x0350, 0x05bd, 0x07e6,
    0x1abf, 0x18e4, 0x1e09, 0x1c52, 0x13d3, 0x1188, 0x1765, 0x153e,
    0x369a, 0x34c1, 0x322c, 0x3077, 0x3ff6, 0x3dad, 0x3b40, 0x391b,
    0x2442, 0x2619, 0x20f4, 0x22af, 0x2d2e, 0x2f75, 0x2998, 0x2bc3,
    0x132a, 0x1171, 0x179c, 0x15c7, 0x1a46, 0x181d, 0x1ef0, 0x1cab,
    0x01f2, 0x03a9, 0x0544, 0x071f, 0x089e, 0x0ac5, 0x0c28, 0x0e73,
    0x3fa1, 0x3dfa, 0x3b17, 0x394c, 0x36cd, 0x3496, 0x327b, 0x3020,
    0x2d79, 0x2f22, 0x29cf, 0x2b94, 0x2415, 0x264e, 0x20a3, 0x22f8,
    0x1a11, 0x184a, 0x1ea7, 0x1cfc, 0x137d, 0x1126, 0x17cb, 0x1590,
    0x08c9, 0x0a92, 0x0c7f, 0x0e24, 0x01a5, 0x03fe, 0x0513, 0x0748
};

uint16_t crcTable2f[256] =
{
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
    0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
    0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
    0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
    0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
    0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
    0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
    0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
    0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
    0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
    0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
    0x3E, 0x11, 0x60, 0x4F, 0x82, 0xA1, 0xD0, 0xFF, 0x3A, 0x15, 0x64, 0x4B, 0x86, 0xA5, 0xD4, 0xFB,
    0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x42, 0x6D, 0x1C, 0x33, 0xFE, 0xD1, 0xA0, 0x8F,
    0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xE4
};

const uint8_t utilTopN[9] = { 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };

// Static variable definition for register debugging
bool BatmanBattery::_registerDebugEnabled = false;

// Batman IC command arrays
uint16_t WakeUp[2] = {0x2ad4, 0x0000};
uint16_t Mute[2] = {0x20dd, 0x0000};
uint16_t Unmute[2] = {0x21f2, 0x0000};
uint16_t Snap[2] = {0x2BFB, 0x0000};
uint16_t reqTemp = 0x0E1B;
uint16_t padding = 0x0000;

// Global variables for SPI communication
uint16_t receive1 = 0;
uint16_t receive2 = 0;
float tempval1 = 0;
float tempval2 = 0;
float temp1 = 0;
float temp2 = 0;
uint8_t Fluffer[72]; // Buffer for SPI data
uint8_t count1 = 0;
uint8_t count2 = 0;
uint8_t count3 = 0;
uint8_t LoopTimer1 = 5;

// Balancing configuration
#define cycletime 2  // 2 * 50ms = 100ms cycle
// Balancing hysteresis now comes from datalayer.battery.settings.balancing_hysteresis_mV

#endif

static void print_units(char* header, int value, char* units) {
  logging.print(header);
  logging.print(value);
  logging.print(units);
}

void BatmanBattery::update_values() {
#ifdef BATMAN_BATTERY
  // Run the Batman IC communication loop
  loop();

  // Update datalayer with current values
  datalayer_battery->status.cell_min_voltage_mV = (uint16_t)(CellVMin);
  datalayer_battery->status.cell_max_voltage_mV = (uint16_t)(CellVMax);
  
  // Calculate pack voltage from individual cells
  float total_voltage = 0;
  int valid_cells = 0;
  
  // Clear all cell balancing status first
  for (int i = 0; i < MAX_AMOUNT_CELLS; i++) {
    datalayer_battery->status.cell_balancing_status[i] = false;
  }
  
  for (int chip = 0; chip < 8; chip++) {
    for (int reg = 0; reg < 15; reg++) {
      if (Voltage[chip][reg] > 10) {  // Valid cell voltage (>10mV)
        total_voltage += Voltage[chip][reg];
        if (valid_cells < 108) {  // Maximum cell count
          datalayer_battery->status.cell_voltages_mV[valid_cells] = Voltage[chip][reg];
          
          // Check if this cell is being balanced
          // Use CellBalCmd array to determine balancing status
          if (BalancingEnabled && (CellBalCmd[chip] & (0x01 << reg))) {
            datalayer_battery->status.cell_balancing_status[valid_cells] = true;
          }
        }
        valid_cells++;
      }
    }
  }
  
  // Update pack voltage in dV (decivolts)
  if (valid_cells > 0) {
    datalayer_battery->status.voltage_dV = (uint16_t)(total_voltage / 100);  // Convert mV to dV
  } else {
    datalayer_battery->status.voltage_dV = 3700;  // Fallback
  }

  // Tesla Model 3 battery specifications
  datalayer_battery->info.total_capacity_Wh = 75000;  // 75kWh
  
  // Calculate SOC based on cell voltages (simplified)
  float voltage_ratio = (float)datalayer_battery->status.voltage_dV / 4040.0;  // Ratio to max voltage
  datalayer_battery->status.real_soc = (uint16_t)(voltage_ratio * 10000);  // Convert to 0.01% units
  datalayer_battery->status.remaining_capacity_Wh = (uint32_t)(datalayer_battery->info.total_capacity_Wh * voltage_ratio);
  
  datalayer_battery->status.soh_pptt = 9900;  // 99.00%
  datalayer_battery->status.current_dA = 0;  // 0 A - would need current sensor
  
  // Temperature data
  datalayer_battery->status.temperature_min_dC = (int16_t)(TempMin * 10);  
  datalayer_battery->status.temperature_max_dC = (int16_t)(TempMax * 10);  
  
  datalayer_battery->status.max_discharge_power_W = 200000;  // 200kW
  datalayer_battery->status.max_charge_power_W = 120000;  // 120kW

#else
  // Fallback to fake values if BATMAN_BATTERY not defined
  datalayer_battery->status.real_soc = 5000;  // 50.00%
  datalayer_battery->status.soh_pptt = 9900;  // 99.00%
  datalayer_battery->status.voltage_dV = 3700;  // 370.0V
  datalayer_battery->status.current_dA = 0;  // 0 A
  datalayer_battery->info.total_capacity_Wh = 30000;  // 30kWh
  datalayer_battery->status.remaining_capacity_Wh = 15000;  // 15kWh
  datalayer_battery->status.cell_max_voltage_mV = 3596;
  datalayer_battery->status.cell_min_voltage_mV = 3500;
  datalayer_battery->status.temperature_min_dC = 50;  // 5.0°C
  datalayer_battery->status.temperature_max_dC = 60;  // 6.0°C
  datalayer_battery->status.max_discharge_power_W = 5000;  // 5kW
  datalayer_battery->status.max_charge_power_W = 5000;  // 5kW

  for (int i = 0; i < 96; ++i) {
    datalayer_battery->status.cell_voltages_mV[i] = 3700 + random(-20, 21);
    datalayer_battery->status.cell_balancing_status[i] = false;  // No balancing in fallback mode
  }
#endif

  // Mark that we have valid battery data
  datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;

#ifdef DEBUG_LOG
  logging.println("Tesla Model 3 Batman IC Values going to inverter");
  print_units("SOH%: ", (datalayer_battery->status.soh_pptt * 0.01), "% ");
  print_units(", SOC%: ", (datalayer_battery->status.reported_soc * 0.01), "% ");
  print_units(", Voltage: ", (datalayer_battery->status.voltage_dV * 0.1), "V ");
  print_units(", Max discharge power: ", datalayer_battery->status.max_discharge_power_W, "W ");
  print_units(", Max charge power: ", datalayer_battery->status.max_charge_power_W, "W ");
  print_units(", Max temp: ", (datalayer_battery->status.temperature_max_dC * 0.1), "°C ");
  print_units(", Min temp: ", (datalayer_battery->status.temperature_min_dC * 0.1), "°C ");
  print_units(", Max cell voltage: ", datalayer_battery->status.cell_max_voltage_mV, "mV ");
  print_units(", Min cell voltage: ", datalayer_battery->status.cell_min_voltage_mV, "mV ");
  logging.println("");
#endif
}

void BatmanBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
}

void BatmanBattery::transmit_can(unsigned long currentMillis) {
  // Send 100ms CAN Message
  if (currentMillis - previousMillis100 >= INTERVAL_100_MS) {
    previousMillis100 = currentMillis;
    // Put fake messages here in case you want to test sending CAN
    //transmit_can_frame(&TEST, can_interface);
  }
}

void BatmanBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  // Tesla Model 3 battery pack specifications
  datalayer_battery->info.max_design_voltage_dV =
      4040;  // 404.4V, over this, charging is not possible (goes into forced discharge)
  datalayer_battery->info.min_design_voltage_dV = 2450;  // 245.0V under this, discharging further is disabled
  datalayer_battery->info.number_of_cells = 96;  // Tesla Model 3: 2x23 + 2x25 = 96 cells

#ifdef BATMAN_BATTERY
  // Initialize Batman IC
  logging.println("Initializing Tesla Model 3 Batman IC Battery Management...");
  BatStart();
  logging.println("Tesla Model 3 Batman IC initialized successfully");
#endif

  if (allows_contactor_closing) {
    *allows_contactor_closing = true;
  }
}

#ifdef BATMAN_BATTERY

void BatmanBattery::BatStart() {
    Serial.println("\n=== Initializing BMB SPI Interface ===");
    
    // Initialize member variables
    spi_dev = NULL;
    ChipNum = 8; // 4 BMBs x 2 chips each = 8 chips total for Tesla Model 3
    CellVMax = 0;
    CellVMin = NAN;
    TempMax = 0;
    TempMin = 1000;
    BalanceFlag = false;
    BalancingEnabled = true;  // Enable balancing phase cycling by default
    BalancingStateChanged = false;  // Initialize balancing state change flag
    BmbTimeout = true;
    LoopState = 0;
    LoopRanCnt = 0;
    WakeCnt = 0;
    WaitCnt = 0;
    IdleCnt = 0;
    SendDelay = 125;  // 125 microseconds
    lasttime = 0;
    Cell1start = 0;
    Cell2start = 0;
    BalancePhase = 0;  // Start with measurement only phase
    LastCellBalancing = 0;  // Initialize balancing count
    
    // Initialize arrays
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 15; j++) {
            Voltage[i][j] = 0;
        }
        CellBalCmd[i] = 0;
        Temps[i] = 0;
        Temp1[i] = 0;
        Temp2[i] = 0;
        Volts5v[i] = 0;
        ChipV[i] = 0;
        Cfg[i][0] = 0;
        Cfg[i][1] = 0;
        BmbConnected[i] = false;
        LastBmbResponse[i] = millis();
    }
    ActualBmbCount = 0;
    
    Serial.printf("Number of BMB chips: %d\n", ChipNum);
    
    // Initialize ESP32 SPI
    Serial.println("Configuring SPI bus...");
    spi_bus_config_t buscfg = {
        .mosi_io_num = BMB_MOSI,
        .miso_io_num = BMB_MISO,
        .sclk_io_num = BMB_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    Serial.printf("SPI Pins - MOSI: %d, MISO: %d, SCK: %d, CS: %d\n", 
        BMB_MOSI, BMB_MISO, BMB_SCK, BMB_CS);
    esp_err_t ret = spi_bus_initialize(BMB_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("SPI bus initialization failed! Error: %d\n", ret);
        return;
    }
    Serial.println("SPI bus initialized successfully");
    
    Serial.println("Configuring SPI device...");
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                  // SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,  // 1 MHz
        .spics_io_num = BMB_CS,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ret = spi_bus_add_device(BMB_SPI_HOST, &devcfg, &spi_dev);
    if (ret != ESP_OK) {
        Serial.printf("Failed to add SPI device! Error: %d\n", ret);
        return;
    }
    Serial.println("SPI device added successfully");
    
    Serial.println("Configuring CS pin...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BMB_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        Serial.printf("CS pin configuration failed! Error: %d\n", ret);
        return;
    }
    gpio_set_level(BMB_CS, 1);  // CS inactive high
    Serial.println("CS pin configured successfully");
    
    // Configure BMB_ENABLE pin to be always low/0
    Serial.println("Configuring BMB_ENABLE pin...");
    gpio_config_t enable_conf = {
        .pin_bit_mask = (1ULL << BMB_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&enable_conf);
    if (ret != ESP_OK) {
        Serial.printf("BMB_ENABLE pin configuration failed! Error: %d\n", ret);
        return;
    }
    gpio_set_level(BMB_ENABLE, 0);  // BMB_ENABLE always low/0
    Serial.println("BMB_ENABLE pin configured to low state");
    
    Serial.println("=== BMB SPI Interface Initialization Complete ===\n");

    // Add SPI communication check here
    checkSPIConnection();
}

void BatmanBattery::loop() {
    StateMachine();
}

void BatmanBattery::StateMachine() {
    // Debug: Track state machine cycle timing
    static unsigned long lastCycleStart = 0;
    
    if (LoopState == 0 && millis() - lastCycleStart > 200) {
        // New cycle starting
        if (lastCycleStart > 0) {
            Serial.printf("BMS Cycle: %lu ms (Phase %d)\n", 
                         millis() - lastCycleStart, BalancePhase);
        }
        lastCycleStart = millis();
    }
    
    switch (LoopState) {
        case 0: // First state check if there is time out of comms requiring full wake
        {
            if(BmbTimeout == true) {
                WakeUP(); // Send wake up for all BMB boards
                LoopState++;
            } else {
                LoopState++;
            }
            break;
        }

        case 1:
        {
            // Configuration state: Read auxiliary and configuration data
            IdleWake(); // Unmute
            GetData(0x4D); // Read Aux A. Contains 5v reg voltage in word 1
            GetData(0x50); // Read Cfg
            LoopState++;
            break;
        }

        case 2:
        {
            // Snapshot state: Take snapshot of cell voltages
            IdleWake(); // Unmute
            delayMicroseconds(SendDelay);
            Generic_Send_Once(Snap, 1); // Take a snapshot of the cell voltages
            LoopState++;
            break;
        }

        case 3:
        {
            // Snapshot state: Take another snapshot of cell voltages
            IdleWake(); // Unmute
            delayMicroseconds(SendDelay);
            Generic_Send_Once(Snap, 1); // Take a snapshot of the cell voltages
            LoopState++;
            break;
        }

        case 4:
        {
            // Read state: Read status and cell voltage measurements
            IdleWake(); // Unmute
            delayMicroseconds(SendDelay);
            GetData(0x4F); // Read status reg
            delayMicroseconds(SendDelay);
            GetData(0x4F); // Read status reg
            delayMicroseconds(SendDelay);
            GetData(0x47); // Read A. Contains Cell voltage measurements
            delayMicroseconds(SendDelay);
            GetData(0x48); // Read B. Contains Cell voltage measurements
            delayMicroseconds(SendDelay);
            GetData(0x49); // Read C. Contains Cell voltage measurements
            delayMicroseconds(SendDelay);
            GetData(0x4A); // Read D. Contains Cell voltage measurements
            WriteCfg(); // Additional balancing command update for consistency

            LoopState++;
            break;
        }

        case 5:
        {
            // Read state: Read remaining cell voltages and temperature data
            IdleWake(); // Unmute
            delayMicroseconds(SendDelay);
            GetData(0x4F); // Read status reg
            delayMicroseconds(SendDelay);
            GetData(0x4F); // Read status reg
            delayMicroseconds(SendDelay);
            GetData(0x4B); // Read E. Contains Cell voltage measurements
            delayMicroseconds(SendDelay);
            GetData(0x4C); // Read F. Contains chip total V in word 1.
            delayMicroseconds(SendDelay);
            GetData(0x4D); // Read auxiliary data
            delayMicroseconds(SendDelay);
            GetTempData(); // Request temps

            WriteCfg(); // Send balancing configuration to BMB chips

            LoopState++;
            break;
        }

        case 6:
        {
            // Wake state: Wake up BMBs and verify configuration
            WakeUP(); // Send wake up for all BMB boards
            GetData(0x50); // Read Cfg
            WriteCfg();
            GetData(0x50); // Read Cfg
            Generic_Send_Once(Unmute, 2); // Unmute
            LoopState++;
            break;
        }

        case 7:
        {
            // Process state: Update temperature and voltage measurements
            // Always update temperatures as they're not affected by balancing
            upDateTemps();
            upDateAuxVolts();
            
            // Check if balancing state has changed - force recalculation if needed
            if (BalancingStateChanged) {
                // Force reset to Phase 0 to ensure fresh balancing command calculation
                BalancePhase = 0;
                BalancingStateChanged = false;  // Clear the flag
                Serial.println("BALANCING STATE CHANGED: Forcing Phase 0 and recalculating balancing commands");
            }
            
            // Only process cell voltages during Phase 0 (measurement-only phase) 
            // to ensure stable readings without balance resistor interference
            if(BalancePhase == 0) {
                upDateCellVolts();
                // Add debug info to show when voltage processing occurs
                if (_registerDebugEnabled) {
                    Serial.println("VOLTAGE PROCESSING: Phase 0 - measurement-only phase (stable readings)");
                }
            } else {
                // During balancing phases, still update individual cell voltage parameters for display
                updateIndividualCellVoltageParameters();
                
                if (_registerDebugEnabled) {
                    Serial.print("VOLTAGE SKIPPED: Phase ");
                    Serial.print(BalancePhase);
                    Serial.println(" - balancing active, waiting for measurement phase");
                }
            }
            
            LoopState = 0;
            break;
        }

        case 8: // Waiting State
        {
            // Idle state: Wait for next cycle
            IdleCnt++;

            if(IdleCnt > cycletime) {
                LoopState = 0;
                IdleCnt = 0;
                LoopRanCnt++;
            }
            break;
        }

        default: // Should not get here
        {
            // Error state: Reset to initial state
            break;
        }
    }

    // Update BMB connectivity status and check for timeouts
    updateBmbConnectivity();
}

uint16_t BatmanBattery::spi_xfer(spi_host_device_t host, uint16_t data) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = {0},
        .rx_data = {0}
    };
    
    // Set TX data
    t.tx_data[0] = (data >> 8) & 0xFF;
    t.tx_data[1] = data & 0xFF;
    
    // Perform SPI transaction
    ESP_ERROR_CHECK(spi_device_transmit(spi_dev, &t));
    
    // Return RX data
    return ((t.rx_data[0] << 8) | t.rx_data[1]);
}

// Continue with rest of implementation...
// [The file is getting very long, I'll continue with the key methods]

void BatmanBattery::IdleWake() {
    if(BalanceFlag == true && BalancePhase != 0) {
        // Only mute during active balancing phases (1=even, 2=odd), not during measurement phase (0)
        Generic_Send_Once(Mute, 2); // Mute need to do more when balancing to dig into (Primen_CMD)
        // Add settling delay after mute command - LTC6813 datasheet specifies 100µs minimum
        // Using 200µs to ensure stable readings with faster loop timing
        delayMicroseconds(200);
    } else {
        // During measurement phase (BalancePhase == 0) or when balancing is off, always unmute
        Generic_Send_Once(Unmute, 2); // Unmute
        // Add small delay after unmute as well for consistency
        delayMicroseconds(50);
    }
}

void BatmanBattery::GetData(uint8_t ReqID) {
    // Initialize temporary arrays for command and data processing
    uint8_t tempData[2] = {0};
    uint16_t ReqData[2] = {0};

    // Prepare command data with register ID
    tempData[0] = ReqID;

    // Format command data with CRC
    ReqData[0] = ReqID << 8;
    ReqData[1] = (calcCRC(tempData, 2))<<8;

    // Activate chip select (active low)
    gpio_set_level(BMB_CS, 0);  // CS active low

    // Send command bytes over SPI
    receive1 = spi_xfer(BMB_SPI_HOST, ReqData[0]);  // do a transfer
    receive2 = spi_xfer(BMB_SPI_HOST, ReqData[1]);  // do a transfer

    // Read response data (72 bytes total)
    for (count2 = 0; count2 <= 72; count2 = count2 + 2) {
        receive1 = spi_xfer(BMB_SPI_HOST, padding);  // do a transfer
        Fluffer[count2] = receive1 >> 8;
        Fluffer[count2 + 1] = receive1 & 0xFF;
    }

    // Deactivate chip select
    gpio_set_level(BMB_CS, 1);  // CS inactive high

    // Enhanced register debugging - show raw data when enabled
    if (_registerDebugEnabled) {
        Serial.printf("\n=== BMB Register 0x%02X Raw Data ===\n", ReqID);
        Serial.printf("Raw SPI Response (72 bytes):\n");
        for (int i = 0; i < 72; i += 8) {
            Serial.printf("  ");
            for (int j = 0; j < 8 && (i + j) < 72; j++) {
                Serial.printf("%02X ", Fluffer[i + j]);
            }
            Serial.println();
        }
        Serial.println("======================================");
    }

    uint16_t tempvol = 0;

    switch (ReqID) {
        case 0x47:
            // Read Register A: Contains cell voltage measurements for cells 1-3
            for (int h = 0; h < ChipNum; h++) {
                for (int g = 0; g <= 2; g++) {
                    tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
                    if (tempvol != 0xffff) {
                        Voltage[h][g] = tempvol / 12.5;
                    }
                    
                    // Enhanced debugging for Register A (cells 1-3)
                    if (_registerDebugEnabled) {
                        Serial.printf("  Chip %d, Reg %d: Raw=0x%04X (%5u) -> %.1fmV %s\n", 
                            h, g, tempvol, tempvol, 
                            (tempvol != 0xffff) ? Voltage[h][g] : 0.0f,
                            (tempvol == 0xffff) ? "(INVALID)" : (tempvol == 0) ? "(ZERO/DEAD)" : "(VALID)");
                    }
                }
            }
            if (_registerDebugEnabled) Serial.println();
            
            // Validate BMB connectivity for voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x48:
            // Read Register B: Contains cell voltage measurements for cells 4-6
            for (int h = 0; h < ChipNum; h++) {
                for (int g = 3; g <= 5; g++) {
                    tempvol = Fluffer[1 + (h * 9) + ((g - 3) * 2)] * 256 + Fluffer [0 + (h * 9) + ((g - 3) * 2)];
                    if (tempvol != 0xffff) {
                        Voltage[h][g] = tempvol / 12.5;
                    }
                    
                    // Enhanced debugging for Register B (cells 4-6)
                    if (_registerDebugEnabled) {
                        Serial.printf("  Chip %d, Reg %d: Raw=0x%04X (%5u) -> %.1fmV %s\n", 
                            h, g, tempvol, tempvol, 
                            (tempvol != 0xffff) ? Voltage[h][g] : 0.0f,
                            (tempvol == 0xffff) ? "(INVALID)" : (tempvol == 0) ? "(ZERO/DEAD)" : "(VALID)");
                    }
                }
            }
            if (_registerDebugEnabled) Serial.println();
            
            // Validate BMB connectivity for voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x49:
            // Read Register C: Contains cell voltage measurements for cells 7-9
            for (int h = 0; h < ChipNum; h++) {
                for (int g = 6; g <= 8; g++) {
                    tempvol = Fluffer[1 + (h * 9) + ((g - 6) * 2)] * 256 + Fluffer [0 + (h * 9) + ((g - 6) * 2)];
                    if (tempvol != 0xffff) {
                        Voltage[h][g] = tempvol / 12.5 ;
                    }
                    
                    // Enhanced debugging for Register C (cells 7-9)
                    if (_registerDebugEnabled) {
                        Serial.printf("  Chip %d, Reg %d: Raw=0x%04X (%5u) -> %.1fmV %s\n", 
                            h, g, tempvol, tempvol, 
                            (tempvol != 0xffff) ? Voltage[h][g] : 0.0f,
                            (tempvol == 0xffff) ? "(INVALID)" : (tempvol == 0) ? "(ZERO/DEAD)" : "(VALID)");
                    }
                }
            }
            if (_registerDebugEnabled) Serial.println();
            
            // Validate BMB connectivity for voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x4A:
            // Read Register D: Contains cell voltage measurements for cells 10-12
            for (int h = 0; h < ChipNum; h++) {
                for (int g = 9; g <= 11; g++) {
                    tempvol = Fluffer[1 + (h * 9) + ((g - 9) * 2)] * 256 + Fluffer [0 + (h * 9) + ((g - 9) * 2)];
                    if (tempvol != 0xffff) {
                        Voltage[h][g] = tempvol / 12.5;
                    }
                }
            }
            
            // Validate BMB connectivity for voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x4B:
            // Read Register E: Contains cell voltage measurements for cells 13-15
            for (int h = 0; h < ChipNum; h++) {
                for (int g = 12; g <= 14; g++) {
                    tempvol = Fluffer[1 + (h * 9) + ((g - 12) * 2)] * 256 + Fluffer [0 + (h * 9) + ((g - 12) * 2)];
                    if (tempvol != 0xffff) {
                        Voltage[h][g] = tempvol / 12.5;
                    }
                }
            }
            
            // Validate BMB connectivity for voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x4C:
            // Read Register F: Contains chip total voltage in word 1
            for (int h = 0; h < ChipNum; h++) {
                tempvol = Fluffer[3 + (h * 7)] * 256 + Fluffer [2 + (h * 7)];
                if (tempvol != 0xffff) {
                    ChipV[h] = tempvol;
                }
            }
            
            // Validate BMB connectivity for chip voltage register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x4D:
            // Read Auxiliary A Register: Contains temperature and 5V supply data
            for (int h = 0; h < ChipNum; h++) {
                // Read first word - Internal Temperature 1
                tempvol = Fluffer[1 + (h * 9)] * 256 + Fluffer [0 + (h * 9)];
                if (tempvol != 0xffff) {
                    Temp1[h] = tempvol;  // Store raw temperature value
                }

                // Read second word - 5V Supply Voltage
                tempvol = Fluffer[3 + (h * 9)] * 256 + Fluffer [2 + (h * 9)];
                if (tempvol != 0xffff) {
                    if(h == 0 || h == 3 || h == 5 || h == 7) {
                        Volts5v[h] = rev16(tempvol);  // Reverse byte order for specific chips
                    } else {
                        Volts5v[h] = tempvol;  // Store as is for other chips
                    }
                }

                // Read third word - Internal Temperature 2
                tempvol = Fluffer[5 + (h * 9)] * 256 + Fluffer [4 + (h * 9)];
                if (tempvol != 0xffff) {
                    Temp2[h] = tempvol;  // Store raw temperature value
                }
            }
            
            // Validate BMB connectivity for auxiliary register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        case 0x50:
            // Read Configuration Register: Contains chip configuration data
            for (int h = 0; h < ChipNum; h++) {
                tempvol = Fluffer[0 + (h * 7)] * 256 + Fluffer [1 + (h * 7)];
                if (tempvol != 0xffff) {
                    Cfg[h][0] = tempvol;
                }

                tempvol = Fluffer[2 + (h * 7)] * 256 + Fluffer [3 + (h * 7)];
                if (tempvol != 0xffff) {
                    Cfg[h][1] =  tempvol;
                }
            }
            
            // Validate BMB connectivity for configuration register
            for (int h = 0; h < ChipNum; h++) {
                if (validateBmbResponse(h, ReqID)) {
                    BmbConnected[h] = true;
                    LastBmbResponse[h] = millis();
                }
            }
            break;

        default:
            break;
    }
}

void BatmanBattery::WriteCfg() {
    // CMD(one byte) PEC(one byte)
    uint8_t tempData[6] = {0};
    uint16_t cfgwrt [25] = {0};

    // Set the write configuration command (0x11) with PEC
    cfgwrt[0]= 0x112F;        //CMD

    for (int h = 0; h < 8; h++) { // Write the 8 BMB registers
        // Set configuration register address (0xF3) and initial value (0x00)
        tempData[0]=0xF3;
        tempData[1]=0x00;
        // Note can not be adjacent cells
        // First copy all cells we want to balance
        tempData[2]=CellBalCmd[7-h] & 0x00FF; // Balancing 8-1
        tempData[3]=(CellBalCmd[7-h] & 0xFF00)>>8; // Balancing  16-9

        // Now implement 3-phase balancing: measurement only, even cells, odd cells
        if(BalancePhase == 0) {
            // Phase 0: Measurement only - no balancing (0x00 = 00000000)
            tempData[2] = 0x00;
            tempData[3] = 0x00;
        } else if(BalancePhase == 1) {
            // Phase 1: Balance even cells only (0xAA = 10101010)
            tempData[2] = tempData[2] & 0xAA;
            tempData[3] = tempData[3] & 0xAA;
        } else {
            // Phase 2: Balance odd cells only (0x55 = 01010101)
            tempData[2] = tempData[2] & 0x55;
            tempData[3] = tempData[3] & 0x55;
        }

        // Calculate PEC (Packet Error Code) for data integrity
        uint16_t payPec =0x0010;
        crc14_bytes(4,tempData,&payPec);
        crc14_bits(2,2,&payPec);

        // Pack the configuration data into the write buffer
        cfgwrt[1+h*3] = tempData[1] + (tempData[0] << 8);
        cfgwrt[2+h*3] = tempData[3] + (tempData[2] << 8);
        cfgwrt[3+h*3] = payPec; // Contains the PEC and other data
    }
    
    // Toggle the balancing pattern for next cycle
    // Only cycle through phases if balancing is enabled, otherwise stay in Phase 0 (measurement only)
    if(BalancingEnabled) {
        // Additional safeguard: Don't advance from Phase 0 if balancing state just changed
        // This ensures we get fresh balancing commands before starting balancing phases
        if (BalancingStateChanged && BalancePhase == 0) {
            // Stay in Phase 0 this cycle to recalculate balancing commands
            // The state will be cleared in the next state machine cycle
            Serial.println("WRITECFG: Staying in Phase 0 to recalculate balancing commands");
        } else if(BalancePhase == 0) {
            BalancePhase = 1;
        } else if(BalancePhase == 1) {
            BalancePhase = 2;
        } else {
            BalancePhase = 0;
        }
    } else {
        // Balancing disabled - always stay in Phase 0 (measurement only, no balancing resistors)
        BalancePhase = 0;
    }

    // Send the configuration data over SPI
    gpio_set_level(BMB_CS, 0);  // CS active low

    for (int cnt = 0; cnt < 25; cnt++) {
        receive1 = spi_xfer(BMB_SPI_HOST, cfgwrt[cnt]);  // do a transfer
    }
    gpio_set_level(BMB_CS, 1);  // CS inactive high
}

void BatmanBattery::GetTempData() {
    padding=0x0000;
    gpio_set_level(BMB_CS, 0);  // CS active low
    receive1 = spi_xfer(BMB_SPI_HOST, reqTemp);  // do a transfer
    for (count3 = 0; count3 < 32; count3 ++) {
        receive1 = spi_xfer(BMB_SPI_HOST, padding);  // do a transfer
        if(receive1 != 0xFFFF) {
            if(count3==1) Temps[0]=receive1;//temperature 1
            if(count3==5) Temps[1]=receive1;//temperature 2
            if(count3==9) Temps[2]=receive1;//temperature 3
            if(count3==13) Temps[3]=receive1;//temperature 4
            if(count3==17) Temps[4]=receive1;//temperature 5
            if(count3==21) Temps[5]=receive1;//temperature 6
            if(count3==25) Temps[6]=receive1;//temperature 7
            if(count3==29) Temps[7]=receive1;//temperature 8
        }
    }
    gpio_set_level(BMB_CS, 1);  // CS inactive high
}

void BatmanBattery::WakeUP() {
    for (count1 = 0; count1 <= 4; count1++) {
        gpio_set_level(BMB_CS, 0);  // CS active low
        receive1 = spi_xfer(BMB_SPI_HOST, WakeUp[0]);  // do a transfer
        gpio_set_level(BMB_CS, 1);  // CS inactive high
    }
}

void BatmanBattery::Generic_Send_Once(uint16_t Command[], uint8_t len) {
    gpio_set_level(BMB_CS, 0);  // CS active low
    for (int h = 0; h < len; h++) {
        receive1 = spi_xfer(BMB_SPI_HOST, Command[h]);  // do a transfer
    }
    gpio_set_level(BMB_CS, 1);  // CS inactive high
}

uint8_t BatmanBattery::calcCRC(uint8_t *inData, uint8_t Length) {
    uint8_t CRC8 =  0x10;
    uint8_t crc_temp = 0;
    for (uint8_t i = 0; i<Length; i++) {
        crc_temp = CRC8 ^ inData[i];
        CRC8 = crcTable2f[crc_temp];
    }
    return(CRC8);
}

void BatmanBattery::crc14_bytes( uint8_t len_B, uint8_t *bytes, uint16_t *crcP ) {
    uint8_t pos, idx;

    for ( idx = 0; idx < len_B; idx++ ) {
        pos = (uint8_t)((*crcP >> 6) ^ bytes[idx]);
        *crcP = (uint16_t)( (0x3fff & (*crcP << 8)) ^ (uint16_t)(crc14table[pos]));
    }
}

void BatmanBattery::crc14_bits( uint8_t len_b,uint8_t inB, uint16_t *crcP ) {
    inB = inB & utilTopN[len_b];   // Mask out the bite we don't care about

    *crcP ^= (uint16_t)((inB) << 6); /* move byte into MSB of 14bit CRC */

    while( len_b-- ) {
        if ((*crcP & 0x2000) != 0) /* test for MSB = bit 13 */ {
            *crcP = (uint16_t)((*crcP << 1) ^ crcPolyBmData);
        } else {
            *crcP = (uint16_t)( *crcP << 1);
        }
    }
    *crcP &= 0x3fff;
}

bool BatmanBattery::checkSPIConnection() {
    Serial.println("Checking SPI communication with BMB...");
    
    // Test basic SPI communication
    gpio_set_level(BMB_CS, 0);  // CS active low
    uint16_t response = spi_xfer(BMB_SPI_HOST, 0x4F00);  // Read status register
    gpio_set_level(BMB_CS, 1);  // CS inactive high
    
    if (response == 0xFFFF || response == 0x0000) {
        Serial.println("SPI communication failed: No valid response from BMB.");
        return false;
    }
    
    Serial.printf("SPI communication OK. Status register: 0x%04X\n", response);
    return true;
}

// BMB Connectivity Monitoring Functions
bool BatmanBattery::validateBmbResponse(uint8_t chipIndex, uint8_t reqID) {
    // Check if the response data for this chip contains valid information
    bool hasValidData = false;
    int validReadings = 0;
    int totalReadings = 0;
    
    // For voltage registers (0x47-0x4B), check if we have reasonable voltage readings
    if (reqID >= 0x47 && reqID <= 0x4B) {
        // Check voltage readings for this chip
        for (int reg = 0; reg < 15; reg++) {
            totalReadings++;
            uint16_t voltage = Voltage[chipIndex][reg];
            
            // Valid voltage reading criteria:
            // - Not 0xFFFF (invalid marker)
            // - Not 0x0000 (likely no response)  
            // - Within reasonable battery cell voltage range (1V to 5V = 1000mV to 5000mV)
            if (voltage != 0xFFFF && voltage != 0x0000 && voltage >= 800 && voltage <= 5200) {
                validReadings++;
            }
        }
        
        // Consider BMB connected if we have at least 3 valid voltage readings
        hasValidData = (validReadings >= 3);
    }
    // For auxiliary registers (0x4D, 0x4E), check 5V supply and temperatures
    else if (reqID == 0x4D || reqID == 0x4E) {
        // Check 5V supply voltage (should be around 5000mV)
        uint16_t supply5v = Volts5v[chipIndex];
        if (supply5v != 0xFFFF && supply5v != 0x0000 && supply5v >= 4000 && supply5v <= 6000) {
            hasValidData = true;
        }
    }
    // For configuration registers (0x50), just check if not all 0xFF or 0x00
    else if (reqID == 0x50) {
        if (Cfg[chipIndex][0] != 0xFFFF && Cfg[chipIndex][0] != 0x0000 &&
            Cfg[chipIndex][1] != 0xFFFF && Cfg[chipIndex][1] != 0x0000) {
            hasValidData = true;
        }
    }
    
    return hasValidData;
}

void BatmanBattery::updateBmbConnectivity() {
    unsigned long currentTime = millis();
    ActualBmbCount = 0;
    bool anyBmbConnected = false;
    
    // Check each BMB for timeout and count actually connected ones
    for (int chip = 0; chip < ChipNum; chip++) {
        // Check if this BMB has timed out
        if (BmbConnected[chip] && (currentTime - LastBmbResponse[chip]) > BMB_TIMEOUT_MS) {
            BmbConnected[chip] = false;
            markBmbDataAsStale(chip);
            
            Serial.printf("WARNING: BMB %d disconnected (timeout after %lu ms)\n", 
                         chip, currentTime - LastBmbResponse[chip]);
        }
        
        if (BmbConnected[chip]) {
            ActualBmbCount++;
            anyBmbConnected = true;
        }
    }
    
    // Update BmbTimeout flag - only timeout if NO BMBs are responding for extended period
    static unsigned long lastAnyBmbResponse = 0;
    if (anyBmbConnected) {
        lastAnyBmbResponse = currentTime;
        BmbTimeout = false;
    } else {
        // Only set timeout if no BMBs have responded for a significant period (30 seconds)
        if (currentTime - lastAnyBmbResponse > 30000) {
            BmbTimeout = true;
        }
    }
}

void BatmanBattery::markBmbDataAsStale(uint8_t chipIndex) {
    // Clear voltage data for disconnected BMB
    for (int reg = 0; reg < 15; reg++) {
        Voltage[chipIndex][reg] = 0; // Mark as no data
    }
    
    // Clear auxiliary data
    Volts5v[chipIndex] = 0;
    ChipV[chipIndex] = 0;
    Temps[chipIndex] = 0;
    Temp1[chipIndex] = 0;
    Temp2[chipIndex] = 0;
    
    // Clear configuration data
    Cfg[chipIndex][0] = 0;
    Cfg[chipIndex][1] = 0;
    
    // Clear balancing commands for this chip
    CellBalCmd[chipIndex] = 0;
    
    Serial.printf("Marked data as stale for disconnected BMB chip %d\n", chipIndex);
}

void BatmanBattery::upDateCellVolts(void) {
    // Initialize tracking variables for cell monitoring
    uint8_t Xr = 0; // BMB number
    uint8_t Yc = 0; // Cell voltage register number
    uint8_t hc = 0; // Cells present per chip
    uint8_t h = 0;  // Spot value index
    uint16_t CellBalancing = 0;

    // Reset balancing state and voltage tracking
    BalanceFlag = false;
    CellVMax = 0;
    CellVMin = NAN;

    // Clear all cell balancing commands
    for(uint8_t L =0; L < 8; L++) {
        CellBalCmd[L] = 0;
    }
    
    // Debug: Track when we're recalculating after balancing state change
    static bool lastBalancingEnabled = false;
    if (lastBalancingEnabled != BalancingEnabled) {
        Serial.printf("BALANCING RECALCULATION: BalancingEnabled changed from %s to %s\n", 
                     lastBalancingEnabled ? "true" : "false", 
                     BalancingEnabled ? "true" : "false");
        lastBalancingEnabled = BalancingEnabled;
    }

    // Process all cells across all BMB chips
    while (h <= 108) {
        if(Yc < 15) { // Check actual measurement present (include register 14)
            if(Voltage[Xr][Yc] > 10) { // Check actual measurement present
                // Check if this BMB is currently connected
                if (!BmbConnected[Xr]) {
                    h++; // Still increment cell index to maintain parameter mapping
                    // Skip min/max and balancing calculations for offline BMB
                } else {
                    // BMB is online - process normally
                    float cellVoltage = Voltage[Xr][Yc];
                    
                    // Track maximum cell voltage and its position
                    if (CellVMax < cellVoltage) {
                        CellVMax = cellVoltage;
                    }
                    // Track minimum cell voltage and its position
                    // Handle NaN initialization properly
                    if (std::isnan(CellVMin) || CellVMin > cellVoltage) {
                        CellVMin = cellVoltage;
                    }

                    // Cell balancing logic:
                    // 1. Check if balancing is enabled
                    // 2. If cell voltage is above minimum + hysteresis, enable balancing
                    // 3. Set corresponding bit in CellBalCmd register using HARDWARE POSITION
                    if(BalancingEnabled) { // Check if balancing phase cycling is enabled
                        if((CellVMin + datalayer.battery.settings.balancing_hysteresis_mV) < cellVoltage) {
                            // Set bit in balancing command register using HARDWARE REGISTER POSITION (Yc)
                            // NOT sequential cell number (h). This ensures balancing commands map to correct hardware registers.
                            CellBalCmd[Xr] = CellBalCmd[Xr] | (0x01 << Yc);
                            CellBalancing++;
                            BalanceFlag = true;
                        }
                    }
                    
                    h++; // Next cell spot value along
                    hc++; // One more cell present
                }
            }

            Yc++; // Next cell along
        } else {
            Yc = 0; // Reset Cell column
            hc = 0; // Reset cell count per chip
            Xr++; // Next BMB
        }

        if(Xr == ChipNum) {
            h = 108;
            break;
        }
    }

    // Store number of cells currently being balanced - ALWAYS update this parameter
    // During Phase 0 (measurement-only), calculate and preserve the balancing count
    // During other phases, use the preserved count so display shows correct balancing status
    if(BalancePhase == 0) {
        // Phase 0: Store the calculated balancing count for use in other phases
        LastCellBalancing = CellBalancing;
    }

    // Debug tracking of initial cell voltages
    if(Cell1start == 0) {
        Cell1start = CellVMax; // Use max voltage for debug
        Cell2start = CellVMin; // Use min voltage for debug
    }

    // Print cell voltage information with hardware position mapping
    Serial.println("\n=== Cell Voltage Information ===");
    int totalCells = 0;
    for (int chip = 0; chip < ChipNum; chip++) {
        for (int reg = 0; reg < 15; reg++) {
            if (Voltage[chip][reg] > 10) { // Cell is present
                totalCells++;
            }
        }
    }
    Serial.printf("Total Cells Present: %d\n", totalCells);
    Serial.printf("Max Cell Voltage: %.3fV\n", CellVMax/1000.0);
    Serial.printf("Min Cell Voltage: %.3fV\n", CellVMin/1000.0);
    Serial.printf("Voltage Delta: %.3fV\n", (CellVMax-CellVMin)/1000.0);
    Serial.printf("Cells Balancing: %d\n", CellBalancing);
    
    // Calculate sum of all cell voltages by dynamically counting all present cells
    float totalCellVoltage = 0;
    int actualCellCount = 0;
    for (int chip = 0; chip < ChipNum; chip++) {
        for (int reg = 0; reg < 15; reg++) {
            if (Voltage[chip][reg] > 10) { // Cell is present
                totalCellVoltage += Voltage[chip][reg];
                actualCellCount++;
            }
        }
    }
    Serial.printf("Total Cell Voltage Sum: %.3fV\n", totalCellVoltage/1000.0);
    Serial.println("==============================\n");
}

void BatmanBattery::upDateAuxVolts(void) {
    // Initialize total pack voltage to 0
    float totalPackVoltage = 0;

    // Process all BMB modules and calculate total pack voltage
    for (int module = 0; module < 4; module++) {
        if (module < (ChipNum / 2)) { // Each module has 2 chips
            // Convert raw ADC values to actual voltages (multiply by 0.001280)
            float chip1Voltage = ChipV[module * 2] * 0.001280;
            float chip2Voltage = ChipV[module * 2 + 1] * 0.001280;
            
            // Add both chip voltages to total pack voltage
            totalPackVoltage += chip1Voltage + chip2Voltage;
        }
    }

    // Calculate average cell voltage from individual cell parameters
    float totalCellVoltage = 0;
    int cellCount = 0;
    
    // Sum all cell voltages that are valid
    for (int chip = 0; chip < ChipNum; chip++) {
        for (int reg = 0; reg < 15; reg++) {
            if (Voltage[chip][reg] > 10) { // Valid cell voltage
                totalCellVoltage += Voltage[chip][reg];
                cellCount++;
            }
        }
    }
    
    // Print auxiliary voltage information
    Serial.println("=== Auxiliary Voltage Information ===");
    Serial.printf("Total Pack Voltage: %.2fV\n", totalPackVoltage);
    if (cellCount > 0) {
        float avgVoltage = totalCellVoltage / cellCount;
        Serial.printf("Average Cell Voltage: %.2fV\n", avgVoltage/1000.0);
    }
    
    // Print 5V supply voltages for debugging
    for (int chip = 0; chip < ChipNum; chip++) {
        if (Volts5v[chip] > 0) {
            Serial.printf("Chip %d 5V Supply: %.2fV\n", chip+1, Volts5v[chip]/1000.0);
        }
    }
    Serial.println("===================================\n");
}

void BatmanBattery::upDateTemps(void) {
    TempMax = 0;
    TempMin = 100;

    // Process temperature data for each chip in the chain
    for (int g = 0; g < ChipNum; g++) {
        // Reverse byte order of temperature value and convert to actual temperature
        tempval1 = rev16(Temps[g]); // Bytes swapped in the 16 bit words
        if (tempval1 == 0) {
            tempval2 = 0;
        } else if (tempval1 >= (1131)) {
            // Temperature calculation for values above 1131
            tempval1 = tempval1-1131;
            tempval2 = tempval1/10;
        } else {
            // Temperature calculation for values below 1131
            tempval1 = 1131-tempval1;
            tempval2 = tempval1/10;
        }

        // Convert internal temperature sensors to Celsius
        // Formula: ((ADC value * 0.01) - 40) for both sensors
        float temp1_c = ((Temp1[g])*0.01)-40;
        float temp2_c = ((Temp2[g])*0.01)-40;

        // Track maximum temperature from both sensors
        if(TempMax < temp1_c) {
            TempMax = temp1_c;
        }
        if(TempMax < temp2_c) {
            TempMax = temp2_c;
        }

        // Track minimum temperature from both sensors
        if(TempMin > temp1_c) {
            TempMin = temp1_c;
        }
        if(TempMin > temp2_c) {
            TempMin = temp2_c;
        }
    }

    // Print temperature information
    Serial.println("=== Temperature Information ===");
    Serial.printf("Max Temperature: %.1f°C\n", TempMax);
    Serial.printf("Min Temperature: %.1f°C\n", TempMin);
    for (int g = 0; g < ChipNum; g++) {
        float temp1_c = ((Temp1[g])*0.01)-40;
        float temp2_c = ((Temp2[g])*0.01)-40;
        Serial.printf("Chip %d - Temp1: %.1f°C, Temp2: %.1f°C\n", 
            g+1, temp1_c, temp2_c);
    }
    Serial.println("=============================\n");
}

void BatmanBattery::updateIndividualCellVoltageParameters(void) {
    // Update individual cell voltage parameters during all phases
    // This function ensures external interfaces get current voltage data even during balancing phases
    // WITHOUT doing any balancing calculations or decisions
    
    uint8_t Xr = 0; // BMB number
    uint8_t Yc = 0; // Cell voltage register number
    uint8_t h = 0;  // Sequential cell index
    
    // Process all cells across all BMB chips - ONLY update parameters, no balancing logic
    while (h <= 108) {
        if(Yc < 15) { // Check actual measurement present (include register 14)
            if(Voltage[Xr][Yc] > 10 && BmbConnected[Xr]) { // Check actual measurement present AND BMB is connected
                h++; // Next cell spot value along
            } else if(Voltage[Xr][Yc] > 10) {
                // Cell register has data but BMB is offline - keep and increment counter
                h++; // Next cell spot value along
            }
            Yc++; // Next cell along
        } else {
            Yc = 0; // Reset Cell column
            Xr++; // Next BMB
        }

        if(Xr == ChipNum) {
            h = 108; // Exit loop
            break;
        }
    }
}

// Helper methods for cell position mapping
BatmanBattery::CellPosition BatmanBattery::getCellHardwarePosition(int sequentialCellNum) const {
    CellPosition pos = {0, 0, false};
    int cellCount = 0;
    
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 15; j++) {
            if (Voltage[i][j] > 10) { // Cell is present
                cellCount++;
                if (cellCount == sequentialCellNum) {
                    pos.chip = i;
                    pos.register_pos = j;
                    pos.valid = true;
                    return pos;
                }
            }
        }
    }
    return pos;
}

int BatmanBattery::getSequentialCellNumber(int chip, int register_pos) const {
    int cellCount = 0;
    
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 15; j++) {
            if (Voltage[i][j] > 10) { // Cell is present
                cellCount++;
                if (i == chip && j == register_pos) {
                    return cellCount;
                }
            }
        }
    }
    return 0;
}

int BatmanBattery::getMinCell() const {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 15; j++) {
            if (Voltage[i][j] == CellVMin) {
                return getSequentialCellNumber(i, j);
            }
        }
    }
    return 0;
}

int BatmanBattery::getMaxCell() const {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 15; j++) {
            if (Voltage[i][j] == CellVMax) {
                return getSequentialCellNumber(i, j);
            }
        }
    }
    return 0;
}

BatmanBattery::BalancingInfo BatmanBattery::getBalancingInfo() const {
    BalancingInfo info = {0, 0, {0}};
    int cellCount = 0;
    int balancingCount = 0;
    
    // Always calculate total cells present for info completeness
    for (int chip = 0; chip < ChipNum; chip++) {
        for (int reg = 0; reg < 15; reg++) {
            if (Voltage[chip][reg] > 10) { // Cell is present
                cellCount++;
            }
        }
    }
    info.totalCells = cellCount;
    
    // Check if balancing is enabled (simplified for battery emulator)
    if (!BalanceFlag) {
        // When balancing is disabled, return info with empty balancing list
        info.balancingCells = 0;
        return info;
    }
    
    float minVoltage = CellVMin;
    float balanceThreshold = minVoltage + datalayer.battery.settings.balancing_hysteresis_mV;
    
    // Reset cell count for balancing logic
    cellCount = 0;
    
    // Scan through all cells to find which ones SHOULD be balanced
    for (int chip = 0; chip < ChipNum; chip++) {
        for (int reg = 0; reg < 15; reg++) {
            if (Voltage[chip][reg] > 10) { // Cell is present
                cellCount++;
                
                // Check if this cell SHOULD be balanced based on voltage threshold
                if (Voltage[chip][reg] > balanceThreshold) {
                    // This cell should be balanced - add it to the list
                    info.balancingCellNumbers[balancingCount] = cellCount;
                    balancingCount++;
                }
            }
        }
    }
    
    info.balancingCells = balancingCount;
    return info;
}

void BatmanBattery::printHardwareMapping() const {
    Serial.println("\n=== COMPLETE BMB Register Debug (All Channels) ===");
    Serial.printf("Number of BMB chips configured: %d\n", ChipNum);
    Serial.printf("Cell validity threshold: >10mV (>0.010V)\n");
    Serial.println("BMB Register Mapping:");
    Serial.println("  0x47 (A): Registers 0-2   (Cells 1-3)");
    Serial.println("  0x48 (B): Registers 3-5   (Cells 4-6)");
    Serial.println("  0x49 (C): Registers 6-8   (Cells 7-9)");
    Serial.println("  0x4A (D): Registers 9-11  (Cells 10-12)");
    Serial.println("  0x4B (E): Registers 12-14 (Cells 13-15)");
    Serial.println();
    
    int sequentialCell = 1;
    int validCells = 0;
    int totalRegisters = 0;
    
    for (int chip = 0; chip < ChipNum; chip++) {
        Serial.printf("┌─── BMB Chip %d (Registers 0-14) ───┐\n", chip);
        bool chipHasValidCells = false;
        
        for (int reg = 0; reg < 15; reg++) {
            uint16_t rawValue = Voltage[chip][reg];
            float voltage = rawValue / 1000.0;
            totalRegisters++;
            
            // Map register to BMB command
            const char* bmbReg = "???";
            if (reg >= 0 && reg <= 2) bmbReg = "0x47";
            else if (reg >= 3 && reg <= 5) bmbReg = "0x48";
            else if (reg >= 6 && reg <= 8) bmbReg = "0x49";
            else if (reg >= 9 && reg <= 11) bmbReg = "0x4A";
            else if (reg >= 12 && reg <= 14) bmbReg = "0x4B";
            
            Serial.printf("│ Reg %2d (%s): %5dmV (%6.3fV) ", reg, bmbReg, rawValue, voltage);
            
            if (rawValue > 10) {
                Serial.printf("-> Cell %2d ✓ VALID", sequentialCell);
                sequentialCell++;
                validCells++;
                chipHasValidCells = true;
            } else if (rawValue == 0) {
                Serial.printf("-> ---- ✗ DEAD/MISSING");
            } else {
                Serial.printf("-> ---- ✗ LOW (<10mV)");
            }
            Serial.println(" │");
        }
        
        Serial.printf("└─ Chip %d Summary: %s ─┘\n", chip, 
            chipHasValidCells ? "HAS VALID CELLS" : "NO VALID CELLS");
        Serial.println();
    }
    
    // Summary statistics
    Serial.println("=== BMB Channel Analysis Summary ===");
    Serial.printf("Total registers scanned: %d\n", totalRegisters);
    Serial.printf("Valid cells found: %d\n", validCells);
    Serial.printf("Dead/damaged channels: %d\n", totalRegisters - validCells);
    Serial.printf("Sequential cell mapping: 1-%d\n", sequentialCell - 1);
    Serial.println("============================================\n");
}

#endif