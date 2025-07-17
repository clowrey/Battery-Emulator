#include "BATMAN-BATTERY.h"
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../include.h"
#include <driver/gpio.h>
#include <SPI.h>
#include <cstring>

// Batman IC command words
static const uint16_t CMD_WAKE_UP[2] = {0x2AD4, 0x0000};
static const uint16_t CMD_UNMUTE[2] = {0x21F2, 0x0000};
static const uint16_t CMD_SNAPSHOT[2] = {0x2BFB, 0x0000};

// CRC14 table for Batman IC packet error checking
static const uint16_t crc14table[256] = {
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

BatmanBattery::BatmanBattery() :
    renderer(*this),
    spi_dev(nullptr),
    state_machine_step(0),
    chip_count(1),  // Default to 1 Batman IC
    balance_phase(0),
    bmb_timeout(true),
    balance_enabled(false),
    last_update_10ms(0),
    last_update_1s(0),
    last_state_machine(0),
    cell_voltage_max(0),
    cell_voltage_min(0),  // Start as 0V (no data)
    temperature_max(0),
    temperature_min(0),   // Start as 0°C (no data)
    cells_present(0),
    cells_balancing(0),
    pack_current_A(0),
    pack_power_W(0) {
    
    // Initialize arrays to 0 (no data available)
    memset(cell_voltages, 0, sizeof(cell_voltages));
    memset(cell_balance_cmd, 0, sizeof(cell_balance_cmd));
    memset(chip_temperatures, 0, sizeof(chip_temperatures));
    memset(chip_5v_supply, 0, sizeof(chip_5v_supply));
    memset(chip_voltages, 0, sizeof(chip_voltages));
}

BatmanBattery::~BatmanBattery() {
    if (spi_dev) {
        spi_bus_remove_device(spi_dev);
        spi_bus_free(BMB_SPI_HOST);
    }
}

void BatmanBattery::setup() {
    Serial.println("Setting up Batman Battery interface...");
    
    // Set battery protocol name for system identification
    strncpy(datalayer.system.info.battery_protocol, Name, 63);
    datalayer.system.info.battery_protocol[63] = '\0';
    
    init_batman_spi();
    // init_as8510();  // Temporarily disabled to test Batman SPI
    
    // Set reasonable defaults for datalayer
    datalayer.battery.status.max_discharge_power_W = 30000;  // 30kW
    datalayer.battery.status.max_charge_power_W = 11000;     // 11kW
    datalayer.battery.info.number_of_cells = 108;            // Tesla Model 3 standard
    datalayer.battery.info.max_design_voltage_dV = 4040;    // 404V
    datalayer.battery.info.min_design_voltage_dV = 2800;    // 280V
    datalayer.battery.info.max_cell_voltage_mV = 4250;      // 4.25V max cell
    datalayer.battery.info.min_cell_voltage_mV = 2700;      // 2.7V min cell
    datalayer.battery.info.chemistry = battery_chemistry_enum::NCA;
    
    // Initialize the system as active
    datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
    datalayer.battery.status.bms_status = ACTIVE;
    
    // Test SPI communication
    Serial.println("Testing Batman IC SPI communication...");
    if (check_spi_communication()) {
        Serial.println("Batman IC SPI communication: OK");
    } else {
        Serial.println("Batman IC SPI communication: FAILED - Check wiring and power");
        bmb_timeout = true;
    }
    
    // Test SPI communication immediately after initialization
    Serial.println("Testing SPI communication...");
    gpio_set_level(BMB_CS, 0);  // CS active
    uint16_t test_response = spi_xfer(0x1234);  // Send test data
    gpio_set_level(BMB_CS, 1);  // CS inactive
    Serial.printf("SPI test complete: sent 0x1234, received 0x%04X\n", test_response);
    
    Serial.println("Batman Battery setup complete");
}

void BatmanBattery::update_values() {
    unsigned long current_time = millis();
    
    // Run state machine every 100ms (matching original loop timing)
    if (current_time - last_state_machine >= STATE_MACHINE_INTERVAL) {
        batman_state_machine();
        last_state_machine = current_time;
    }
    
    // Fast updates every 10ms
    if (current_time - last_update_10ms >= UPDATE_10MS_INTERVAL) {
        // Quick status updates - always update CAN alive status
        datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
        last_update_10ms = current_time;
    }
    
    // Slow updates every 1 second
    if (current_time - last_update_1s >= UPDATE_1S_INTERVAL) {
        read_current_sensor();
        
        // Update main datalayer with our values
        update_datalayer_values();
        
        last_update_1s = current_time;
    }
}

void BatmanBattery::init_batman_spi() {
    Serial.println("Initializing Batman IC SPI interface...");
    
    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = BMB_MOSI,
        .miso_io_num = BMB_MISO,
        .sclk_io_num = BMB_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    
    esp_err_t ret = spi_bus_initialize(BMB_SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        Serial.printf("SPI bus initialization failed: %d (%s)\n", ret, esp_err_to_name(ret));
        datalayer.battery.status.bms_status = FAULT;
        return;
    }
    Serial.println("SPI bus initialized successfully");
    
    // Configure SPI device with manual CS control
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,  // 1MHz
        .spics_io_num = -1,  // Manual CS control
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };
    
    ret = spi_bus_add_device(BMB_SPI_HOST, &dev_config, &spi_dev);
    if (ret != ESP_OK) {
        Serial.printf("SPI device add failed: %d (%s)\n", ret, esp_err_to_name(ret));
        datalayer.battery.status.bms_status = FAULT;
        return;
    }
    Serial.printf("SPI device added successfully, handle: %p\n", spi_dev);
    
    // Configure CS pin for manual control
    gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << BMB_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cs_config);
    gpio_set_level(BMB_CS, 1);  // CS inactive high
    
    Serial.printf("SPI initialized: MOSI=%d, MISO=%d, SCK=%d, CS=%d\n", 
                  BMB_MOSI, BMB_MISO, BMB_SCK, BMB_CS);
    
    // Configure BMB_ENABLE pin (always low)
    gpio_config_t enable_config = {
        .pin_bit_mask = (1ULL << BMB_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&enable_config);
    gpio_set_level(BMB_ENABLE, 0);  // BMB_ENABLE always low
    
    Serial.println("Batman IC SPI initialization complete");
}

void BatmanBattery::init_as8510() {
    Serial.println("Initializing AS8510 current sensor...");
    
    // Initialize VSPI for AS8510
    SPI.begin(AS8510_SCK_PIN, AS8510_MISO_PIN, AS8510_MOSI_PIN, AS8510_CS_PIN);
    pinMode(AS8510_CS_PIN, OUTPUT);
    digitalWrite(AS8510_CS_PIN, HIGH);  // CS idle high
    
    Serial.println("AS8510 current sensor initialization complete");
}

uint16_t BatmanBattery::spi_xfer(uint16_t data) {
    if (!spi_dev) {
        Serial.println("ERROR: spi_dev is null!");
        return 0;
    }
    
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = {0},
        .rx_data = {0}
    };
    
    // Set TX data (matching original implementation)
    t.tx_data[0] = (data >> 8) & 0xFF;
    t.tx_data[1] = data & 0xFF;
    
    Serial.printf("SPI TX: 0x%04X (bytes: %02X %02X)\n", data, t.tx_data[0], t.tx_data[1]);
    
    // Perform SPI transaction
    esp_err_t ret = spi_device_transmit(spi_dev, &t);
    if (ret != ESP_OK) {
        Serial.printf("SPI transmit failed: %d\n", ret);
        return 0;
    }
    
    uint16_t rx_data = ((t.rx_data[0] << 8) | t.rx_data[1]);
    Serial.printf("SPI RX: 0x%04X (bytes: %02X %02X)\n", rx_data, t.rx_data[0], t.rx_data[1]);
    
    // Return RX data (matching original implementation)
    return rx_data;
}

void BatmanBattery::batman_state_machine() {
    Serial.printf("Batman state machine: Step %d\n", state_machine_step);
    
    switch (state_machine_step) {
        case 0:  // Wake up if timeout
            Serial.println("State 0: Check timeout and wake up");
            if (bmb_timeout) {
                batman_wake_up();
            }
            state_machine_step++;
            break;
            
        case 1:  // Read auxiliary data
            Serial.println("State 1: Read auxiliary data");
            batman_idle_wake();
            batman_get_data(0x4D);  // Read Aux A (5V supply)
            batman_get_data(0x50);  // Read Config
            state_machine_step++;
            break;
            
        case 2:  // Take voltage snapshot
            Serial.println("State 2: Take voltage snapshot");
            batman_idle_wake();
            delay(1000);  // CRITICAL: Use 1 second delay as in original
            batman_take_snapshot();
            state_machine_step++;
            break;
            
        case 3:  // Take another voltage snapshot
            Serial.println("State 3: Take another voltage snapshot");
            batman_idle_wake();
            delay(1000);  // CRITICAL: Use 1 second delay as in original
            batman_take_snapshot();
            state_machine_step++;
            break;
            
        case 4:  // Read cell voltages A-D
            Serial.println("State 4: Read cell voltages A-D");
            batman_idle_wake();
            delay(1000);
            batman_get_data(0x4F);  // Read status reg
            delay(1000);
            batman_get_data(0x4F);  // Read status reg
            delay(1000);
            batman_get_data(0x47);  // Read A
            delay(1000);
            batman_get_data(0x48);  // Read B
            delay(1000);
            batman_get_data(0x49);  // Read C
            delay(1000);
            batman_get_data(0x4A);  // Read D
            state_machine_step++;
            break;
            
        case 5:  // Read cell voltages E-F and temperatures
            Serial.println("State 5: Read cell voltages E-F and temperatures");
            batman_idle_wake();
            delay(1000);
            batman_get_data(0x4F);  // Read status reg
            delay(1000);
            batman_get_data(0x4F);  // Read status reg
            delay(1000);
            batman_get_data(0x4B);  // Read E
            delay(1000);
            batman_get_data(0x4C);  // Read F
            delay(1000);
            batman_get_data(0x4D);  // Read Aux A
            delay(1000);
            batman_get_temp_data();  // Request temperatures
            state_machine_step++;
            break;
            
        case 6:  // Wake up and configure
            Serial.println("State 6: Wake up and configure");
            batman_wake_up();
            batman_get_data(0x50);  // Read Config
            batman_write_config();
            batman_get_data(0x50);  // Read Config
            batman_idle_wake();
            state_machine_step++;
            break;
            
        case 7:  // Process data
            Serial.println("State 7: Process data");
            update_cell_voltages();
            update_aux_voltages();
            update_temperatures();
            process_balancing();
            
            state_machine_step = 0;  // Reset to beginning
            bmb_timeout = false;     // Clear timeout flag
            break;
            
        default:
            state_machine_step = 0;
            break;
    }
}

void BatmanBattery::batman_wake_up() {
    // Wake up multiple times as in original implementation
    for (int count1 = 0; count1 <= 4; count1++) {
        gpio_set_level(BMB_CS, 0);  // CS active
        spi_xfer(CMD_WAKE_UP[0]);
        gpio_set_level(BMB_CS, 1);  // CS inactive
    }
    Serial.println("Batman wake up sent 4 times");
}

void BatmanBattery::batman_idle_wake() {
    gpio_set_level(BMB_CS, 0);  // CS active
    spi_xfer(CMD_UNMUTE[0]);
    spi_xfer(CMD_UNMUTE[1]);
    gpio_set_level(BMB_CS, 1);  // CS inactive
    Serial.println("Batman idle wake (unmute) sent");
}

void BatmanBattery::batman_take_snapshot() {
    gpio_set_level(BMB_CS, 0);  // CS active
    spi_xfer(CMD_SNAPSHOT[0]);
    spi_xfer(CMD_SNAPSHOT[1]);
    gpio_set_level(BMB_CS, 1);  // CS inactive
    Serial.println("Batman snapshot command sent");
}

void BatmanBattery::batman_get_temp_data() {
    // Request temperatures using the GetTempData method (matching original implementation)
    static const uint16_t reqTemp = 0x0E1B;  // Temps returned in words 1 and 5
    uint16_t padding = 0x0000;
    
    gpio_set_level(BMB_CS, 0);  // CS active low
    uint16_t receive1 = spi_xfer(reqTemp);  // Send temperature request
    
    for (int count3 = 0; count3 < 32; count3++) {
        receive1 = spi_xfer(padding);  // Read data
        if (receive1 != 0xFFFF) {
            // Store temperature data from specific word positions
            if (count3 == 1) chip_temperatures[0] = receive1;   // temperature 1
            if (count3 == 5) chip_temperatures[1] = receive1;   // temperature 2
            if (count3 == 9) chip_temperatures[2] = receive1;   // temperature 3
            if (count3 == 13) chip_temperatures[3] = receive1;  // temperature 4
            if (count3 == 17) chip_temperatures[4] = receive1;  // temperature 5
            if (count3 == 21) chip_temperatures[5] = receive1;  // temperature 6
            if (count3 == 25) chip_temperatures[6] = receive1;  // temperature 7
            if (count3 == 29) chip_temperatures[7] = receive1;  // temperature 8
        }
    }
    
    gpio_set_level(BMB_CS, 1);  // CS inactive high
    
    Serial.printf("Temperature data: T1=%d, T2=%d, T3=%d, T4=%d\n", 
                  chip_temperatures[0], chip_temperatures[1], 
                  chip_temperatures[2], chip_temperatures[3]);
}

void BatmanBattery::batman_get_data(uint8_t reg_id) {
    // Initialize based on original GetData implementation
    uint8_t temp_data[2] = {0};
    uint16_t req_data[2] = {0};
    uint8_t response_buffer[74];  // 72 bytes + 2 extra for safety
    
    // Prepare command data with register ID (matching original)
    temp_data[0] = reg_id;
    
    // Format command data with CRC (matching original)
    req_data[0] = reg_id << 8;
    req_data[1] = (calc_crc(temp_data, 2)) << 8;
    
    // Activate chip select (active low) - matching original
    gpio_set_level(BMB_CS, 0);
    
    // Send command bytes over SPI (matching original)
    uint16_t receive1 = spi_xfer(req_data[0]);
    uint16_t receive2 = spi_xfer(req_data[1]);
    
    // Read response data (72 bytes total) - matching original loop
    for (int count2 = 0; count2 <= 72; count2 = count2 + 2) {
        receive1 = spi_xfer(0x0000);  // Send padding
        response_buffer[count2] = receive1 >> 8;
        response_buffer[count2 + 1] = receive1 & 0xFF;
    }
    
    // Deactivate chip select - matching original
    gpio_set_level(BMB_CS, 1);
    
    // Add debug output
    Serial.printf("Batman IC: Read reg 0x%02X - First 8 bytes: ", reg_id);
    for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", response_buffer[i]);
    }
    Serial.println();
    
    // Check if we're getting all 0xFF (no response from Batman IC)
    bool all_ff = true;
    for (int i = 0; i < 8; i++) {
        if (response_buffer[i] != 0xFF) {
            all_ff = false;
            break;
        }
    }
    
    if (all_ff) {
        Serial.printf("WARNING: Batman IC not responding (all 0xFF) for register 0x%02X\n", reg_id);
        bmb_timeout = true;
        
        // Clear cell voltages if this is a voltage register
        if (reg_id >= 0x47 && reg_id <= 0x4C) {
            Serial.println("Clearing cell voltages due to communication failure");
            memset(cell_voltages, 0, sizeof(cell_voltages));
        }
        return;  // Don't process invalid data
    }
    
    // Process the response based on register ID
    process_register_data(reg_id, response_buffer);
}

void BatmanBattery::batman_write_config() {
    // CMD(one byte) PEC(one byte)
    uint8_t temp_data[6] = {0};
    uint16_t cfgwrt[25] = {0};
    
    // Set the write configuration command (0x11) with PEC
    cfgwrt[0] = 0x112F;  // CMD
    
    for (int h = 0; h < chip_count; h++) {
        // Set configuration register address (0xF3) and initial value (0x00)
        temp_data[0] = 0xF3;
        temp_data[1] = 0x00;
        
        // Copy all cells we want to balance
        temp_data[2] = cell_balance_cmd[chip_count - 1 - h] & 0x00FF;  // balancing 8-1
        temp_data[3] = (cell_balance_cmd[chip_count - 1 - h] & 0xFF00) >> 8;  // balancing 16-9
        
        // Implement 3-phase balancing: measurement only, even cells, odd cells
        if (balance_phase == 0) {
            // Phase 0: Measurement only - no balancing (0x00 = 00000000)
            temp_data[2] = 0x00;
            temp_data[3] = 0x00;
        } else if (balance_phase == 1) {
            // Phase 1: Balance even cells only (0xAA = 10101010)
            temp_data[2] = temp_data[2] & 0xAA;
            temp_data[3] = temp_data[3] & 0xAA;
        } else {
            // Phase 2: Balance odd cells only (0x55 = 01010101)
            temp_data[2] = temp_data[2] & 0x55;
            temp_data[3] = temp_data[3] & 0x55;
        }
        
        // Calculate PEC (Packet Error Code) for data integrity
        uint16_t pay_pec = 0x0010;
        crc14_bytes(4, temp_data, &pay_pec);
        crc14_bits(2, 2, &pay_pec);
        
        // Pack the configuration data into the write buffer
        cfgwrt[1 + h * 3] = temp_data[1] + (temp_data[0] << 8);
        cfgwrt[2 + h * 3] = temp_data[3] + (temp_data[2] << 8);
        cfgwrt[3 + h * 3] = pay_pec;
    }
    
    // Send the configuration data over SPI
    gpio_set_level(BMB_CS, 0);  // CS active low
    for (int cnt = 0; cnt < 25; cnt++) {
        spi_xfer(cfgwrt[cnt]);
    }
    gpio_set_level(BMB_CS, 1);  // CS inactive high
    
    delayMicroseconds(125);
}

void BatmanBattery::read_current_sensor() {
    // AS8510 current sensor reading using Arduino SPI
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(AS8510_CS_PIN, LOW);
    
    // Read current data registers (0x00 and 0x01)
    SPI.transfer(0x00);  // Current Data 1 register
    uint8_t current_data1 = SPI.transfer(0x00);
    
    digitalWrite(AS8510_CS_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(AS8510_CS_PIN, LOW);
    
    SPI.transfer(0x01);  // Current Data 2 register
    uint8_t current_data2 = SPI.transfer(0x00);
    
    digitalWrite(AS8510_CS_PIN, HIGH);
    SPI.endTransaction();
    
    // Combine the two bytes to form 16-bit ADC value
    int16_t raw_adc = (current_data1 << 8) | current_data2;
    
    // Convert to current using shunt resistance
    // AS8510 has 16-bit ADC, reference voltage is typically 2.5V
    // Current = (ADC_Value / 65536) * Vref / Shunt_Resistance
    float vref = 2.5;  // Reference voltage
    float adc_voltage = (raw_adc / 65536.0) * vref;
    pack_current_A = adc_voltage / SHUNT_RESISTANCE;
    
    // Calculate power
    if (cells_present > 0) {
        float pack_voltage = datalayer.battery.status.voltage_dV / 10.0;  // Convert dV to V
        pack_power_W = pack_voltage * pack_current_A;
    }
}

void BatmanBattery::update_cell_voltages() {
    // Process cell voltage data and calculate statistics
    cell_voltage_max = 0;
    cell_voltage_min = 5000;  // Start high to find actual minimum
    cells_present = 0;
    float total_voltage = 0;
    
    Serial.println("Updating cell voltages from raw data:");
    
    // Clear the datalayer cell voltage array first
    memset(datalayer.battery.status.cell_voltages_mV, 0, sizeof(datalayer.battery.status.cell_voltages_mV));
    
    // Iterate through all chips and cells, storing only valid cells sequentially
    int cell_index = 0;
    for (int chip = 0; chip < chip_count; chip++) {
        for (int cell = 0; cell < 15; cell++) {
            uint16_t voltage = cell_voltages[chip][cell];
            if (voltage > 1000) {  // Valid cell voltage (>1V, reasonable threshold)
                cells_present++;
                total_voltage += voltage;
                
                Serial.printf("  Chip %d Cell %d: %d mV\n", chip, cell, voltage);
                
                // Store individual cell voltage in datalayer sequentially
                if (cell_index < MAX_AMOUNT_CELLS) {
                    datalayer.battery.status.cell_voltages_mV[cell_index] = voltage;
                    cell_index++;
                }
                
                if (voltage > cell_voltage_max) {
                    cell_voltage_max = voltage;
                }
                if (voltage < cell_voltage_min) {
                    cell_voltage_min = voltage;
                }
            }
        }
    }
    
    // Update datalayer cell count and voltages
    datalayer.battery.info.number_of_cells = cells_present;
    if (cells_present > 0) {
        datalayer.battery.status.voltage_dV = total_voltage / 100;  // Convert mV to dV
        datalayer.battery.status.cell_max_voltage_mV = cell_voltage_max;
        datalayer.battery.status.cell_min_voltage_mV = cell_voltage_min;
        Serial.printf("Found %d cells, Pack: %.1fV, Min: %dmV, Max: %dmV\n", 
                     cells_present, total_voltage/1000.0, cell_voltage_min, cell_voltage_max);
    } else {
        // No valid cells detected - reset to 0 for consistency
        datalayer.battery.status.voltage_dV = 0;
        datalayer.battery.status.cell_max_voltage_mV = 0;
        datalayer.battery.status.cell_min_voltage_mV = 0;
        // Reset internal tracking variables to 0 when no cells present
        cell_voltage_max = 0;
        cell_voltage_min = 0;
        Serial.println("No valid cells detected!");
    }
}

void BatmanBattery::update_aux_voltages() {
    // Update 5V supply voltages and chip voltages
    for (int i = 0; i < chip_count; i++) {
        // Process chip supply voltages
        // Convert raw 5V supply values to actual voltages
        if (chip_5v_supply[i] > 0) {
            // AS8510 conversion: raw value / 12.5 = voltage in mV
            float supply_voltage_mv = chip_5v_supply[i] / 12.5;
            // Store in mV for consistency with other voltage measurements
            chip_5v_supply[i] = supply_voltage_mv;
        }
        
        // Process chip total voltages
        if (chip_voltages[i] > 0) {
            // Convert raw chip voltage to actual voltage
            // Typical conversion factor for Batman IC chip voltage
            float chip_voltage_mv = chip_voltages[i] * 1.28;  // Convert to mV
            chip_voltages[i] = chip_voltage_mv;
        }
    }
}

void BatmanBattery::update_temperatures() {
    // Process temperature data
    float temp_max_found = -1000;  // Start very low to find actual maximum
    float temp_min_found = 1000;   // Start high to find actual minimum
    bool temp_data_found = false;
    
    for (int i = 0; i < chip_count; i++) {
        if (chip_temperatures[i] > 0) {
            // Convert raw temperature values
            float temp = chip_temperatures[i] * 0.1;  // Example conversion
            temp_data_found = true;
            
            if (temp > temp_max_found) {
                temp_max_found = temp;
            }
            if (temp < temp_min_found) {
                temp_min_found = temp;
            }
        }
    }
    
    // Update internal temperature tracking and datalayer
    if (temp_data_found) {
        temperature_max = temp_max_found;
        temperature_min = temp_min_found;
        datalayer.battery.status.temperature_max_dC = temperature_max * 10;
        datalayer.battery.status.temperature_min_dC = temperature_min * 10;
    } else {
        // No valid temperature data - set everything to 0
        temperature_max = 0;
        temperature_min = 0;
        datalayer.battery.status.temperature_max_dC = 0;
        datalayer.battery.status.temperature_min_dC = 0;
    }
}

void BatmanBattery::process_balancing() {
    cells_balancing = 0;
    
    // Clear the datalayer cell balancing status array first
    memset(datalayer.battery.status.cell_balancing_status, 0, sizeof(datalayer.battery.status.cell_balancing_status));
    
    if (!balance_enabled) {
        return;
    }
    
    // Calculate which cells need balancing using sequential indexing
    int cell_index = 0;
    for (int chip = 0; chip < chip_count; chip++) {
        cell_balance_cmd[chip] = 0;
        
        for (int cell = 0; cell < 15; cell++) {
            uint16_t voltage = cell_voltages[chip][cell];
            if (voltage > 1000) {  // Valid cell (>1V)
                if (voltage > (cell_voltage_min + BALANCE_HYSTERESIS_MV)) {
                    // This cell needs balancing
                    cell_balance_cmd[chip] |= (1 << cell);
                    cells_balancing++;
                    
                    // Update datalayer cell balancing status using sequential index
                    if (cell_index < MAX_AMOUNT_CELLS) {
                        datalayer.battery.status.cell_balancing_status[cell_index] = true;
                    }
                }
                cell_index++;  // Increment for each valid cell
            }
        }
    }
}

void BatmanBattery::update_datalayer_values() {
    // Update main datalayer with our calculated values
    datalayer.battery.status.real_soc = calculate_soc();
    datalayer.battery.status.current_dA = pack_current_A * 10;  // Convert A to dA
    datalayer.battery.status.active_power_W = pack_power_W;
    
    // CRITICAL: Tell the system that the battery is still alive
    datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
    
    // Set BMS status - check for communication timeout and cell voltage limits
    if (bmb_timeout) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.println("BMS FAULT: Communication timeout with Batman IC");
    } else if (cells_present == 0) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.println("BMS FAULT: No cells detected");
    } else if (cell_voltage_max == 0 || cell_voltage_min == 0) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.println("BMS FAULT: No valid cell voltage data");
    } else if (cell_voltage_max >= MAX_CELL_VOLTAGE_MV) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.printf("BMS FAULT: Cell overvoltage detected: %.0fmV\n", cell_voltage_max);
    } else if (cell_voltage_min <= MIN_CELL_VOLTAGE_MV) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.printf("BMS FAULT: Cell undervoltage detected: %.0fmV\n", cell_voltage_min);
    } else if (temperature_max != 0 && temperature_max > 60.0) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.printf("BMS FAULT: Overtemperature detected: %.1f°C\n", temperature_max);
    } else if (temperature_min != 0 && temperature_min < -20.0) {
        datalayer.battery.status.bms_status = FAULT;
        Serial.printf("BMS FAULT: Undertemperature detected: %.1f°C\n", temperature_min);
    } else {
        datalayer.battery.status.bms_status = ACTIVE;
    }
    
    // Calculate power limits based on cell voltages and current SOC
    calculate_power_limits();
}

uint16_t BatmanBattery::calculate_soc() {
    // Simple voltage-based SOC calculation
    // In a real implementation, this would use coulomb counting with AS8510
    if (cells_present == 0 || cell_voltage_max == 0 || cell_voltage_min == 0) {
        return 0;  // No valid data available
    }
    
    float avg_cell_voltage = (cell_voltage_max + cell_voltage_min) / 2.0;
    
    // Simple linear mapping from 3.0V to 4.2V = 0% to 100%
    if (avg_cell_voltage <= 3000) return 0;
    if (avg_cell_voltage >= 4200) return 10000;  // 100.00%
    
    return (uint16_t)((avg_cell_voltage - 3000) * 10000 / 1200);
}

void BatmanBattery::calculate_power_limits() {
    // Calculate charge/discharge power limits based on cell voltages and temperature
    uint16_t max_charge_power = 11000;   // 11kW default
    uint16_t max_discharge_power = 30000; // 30kW default
    
    // Only adjust limits if we have valid voltage data
    if (cell_voltage_max != 0 && cell_voltage_min != 0) {
        // Reduce power if cells are near voltage limits
        if (cell_voltage_max > 4100) {  // 4.1V
            max_charge_power = max_charge_power * (4200 - cell_voltage_max) / 100;
        }
        
        if (cell_voltage_min < 3200) {  // 3.2V
            max_discharge_power = max_discharge_power * (cell_voltage_min - 2700) / 500;
        }
    }
    
    // Only adjust for temperature if we have valid temperature data
    if (temperature_max != 0 && temperature_min != 0) {
        // Reduce power if temperature is extreme
        if (temperature_max > 45.0 || temperature_min < -10.0) {
            max_charge_power /= 2;
            max_discharge_power /= 2;
        }
    }
    
    // If no valid data, severely limit power for safety
    if (cells_present == 0) {
        max_charge_power = 0;
        max_discharge_power = 0;
    }
    
    datalayer.battery.status.max_charge_power_W = max_charge_power;
    datalayer.battery.status.max_discharge_power_W = max_discharge_power;
}

void BatmanBattery::process_register_data(uint8_t reg_id, uint8_t* data) {
    // Process the register data based on register ID - matching original implementation exactly
    uint16_t tempvol = 0;
    
    switch (reg_id) {
        case 0x47:  // Read Register A: Contains cell voltage measurements for cells 1-3
            Serial.printf("Processing 0x47 (cells 0-2)\n");
            for (int h = 0; h < chip_count; h++) {
                for (int g = 0; g <= 2; g++) {
                    // Original format: little-endian (low byte first, then high byte)
                    tempvol = data[1 + (h * 9) + (g * 2)] * 256 + data[0 + (h * 9) + (g * 2)];
                    if (tempvol != 0xffff) {
                        // CRITICAL FIX: Apply the same conversion as original implementation
                        cell_voltages[h][g] = tempvol / 12.5;
                        Serial.printf("  Chip %d Cell %d: Raw=0x%04X -> %.1f mV\n", h, g, tempvol, cell_voltages[h][g]);
                    }
                }
            }
            break;
            
        case 0x48:  // Read Register B: Contains cell voltage measurements for cells 4-6
            Serial.printf("Processing 0x48 (cells 3-5)\n");
            for (int h = 0; h < chip_count; h++) {
                for (int g = 3; g <= 5; g++) {
                    tempvol = data[1 + (h * 9) + ((g - 3) * 2)] * 256 + data[0 + (h * 9) + ((g - 3) * 2)];
                    if (tempvol != 0xffff) {
                        // CRITICAL FIX: Apply the same conversion as original implementation
                        cell_voltages[h][g] = tempvol / 12.5;
                        Serial.printf("  Chip %d Cell %d: Raw=0x%04X -> %.1f mV\n", h, g, tempvol, cell_voltages[h][g]);
                    }
                }
            }
            break;
            
        case 0x49:  // Read Register C: Contains cell voltage measurements for cells 7-9
            Serial.printf("Processing 0x49 (cells 6-8)\n");
            for (int h = 0; h < chip_count; h++) {
                for (int g = 6; g <= 8; g++) {
                    tempvol = data[1 + (h * 9) + ((g - 6) * 2)] * 256 + data[0 + (h * 9) + ((g - 6) * 2)];
                    if (tempvol != 0xffff) {
                        // CRITICAL FIX: Apply the same conversion as original implementation
                        cell_voltages[h][g] = tempvol / 12.5;
                        Serial.printf("  Chip %d Cell %d: Raw=0x%04X -> %.1f mV\n", h, g, tempvol, cell_voltages[h][g]);
                    }
                }
            }
            break;
            
        case 0x4A:  // Read Register D: Contains cell voltage measurements for cells 10-12
            Serial.printf("Processing 0x4A (cells 9-11)\n");
            for (int h = 0; h < chip_count; h++) {
                for (int g = 9; g <= 11; g++) {
                    tempvol = data[1 + (h * 9) + ((g - 9) * 2)] * 256 + data[0 + (h * 9) + ((g - 9) * 2)];
                    if (tempvol != 0xffff) {
                        // CRITICAL FIX: Apply the same conversion as original implementation
                        cell_voltages[h][g] = tempvol / 12.5;
                        Serial.printf("  Chip %d Cell %d: Raw=0x%04X -> %.1f mV\n", h, g, tempvol, cell_voltages[h][g]);
                    }
                }
            }
            break;
            
        case 0x4B:  // Read Register E: Contains cell voltage measurements for cells 13-15
            Serial.printf("Processing 0x4B (cells 12-14)\n");
            for (int h = 0; h < chip_count; h++) {
                for (int g = 12; g <= 14; g++) {
                    tempvol = data[1 + (h * 9) + ((g - 12) * 2)] * 256 + data[0 + (h * 9) + ((g - 12) * 2)];
                    if (tempvol != 0xffff) {
                        // CRITICAL FIX: Apply the same conversion as original implementation
                        cell_voltages[h][g] = tempvol / 12.5;
                        Serial.printf("  Chip %d Cell %d: Raw=0x%04X -> %.1f mV\n", h, g, tempvol, cell_voltages[h][g]);
                    }
                }
            }
            break;
            
        case 0x4C:  // Read Register F: Contains chip total voltage in word 1
            Serial.printf("Processing 0x4C (chip voltages)\n");
            for (int h = 0; h < chip_count; h++) {
                tempvol = data[3 + (h * 7)] * 256 + data[2 + (h * 7)];
                if (tempvol != 0xffff) {
                    chip_voltages[h] = tempvol;
                    Serial.printf("  Chip %d total voltage: %d\n", h, tempvol);
                }
            }
            break;
            
        case 0x4D:  // Aux A - 5V supply
            process_aux_data(data);
            break;
            
        case 0x4E:  // Aux B - Temperatures
            process_temperature_data(data);
            break;
            
        case 0x50:  // Config register
            // Process configuration data
            break;
            
        default:
            break;
    }
}

void BatmanBattery::process_cell_voltage_data(uint8_t reg_id, uint8_t* data) {
    // Extract cell voltage data from the register response
    // Based on the context implementation pattern
    int base_cell = (reg_id - 0x47) * 3;  // Calculate base cell index
    
    Serial.printf("Processing cell voltages for reg 0x%02X, base_cell=%d\n", reg_id, base_cell);
    
    // Process data in groups of 6 bytes (3 cells, 2 bytes each)
    for (int i = 0; i < 3 && (base_cell + i) < 15; i++) {
        // Extract 16-bit voltage value from data buffer
        // Data format: chip0_cell0_high, chip0_cell0_low, chip0_cell1_high, chip0_cell1_low, etc.
        uint16_t voltage = (data[i*2] << 8) | data[i*2 + 1];
        
        // Store in chip 0 (assuming single chip for now)
        if (base_cell + i < 15) {
            cell_voltages[0][base_cell + i] = voltage;
            Serial.printf("  Cell %d: %d mV\n", base_cell + i, voltage);
        }
    }
}


void BatmanBattery::process_aux_data(uint8_t* data) {
    // Process auxiliary data (5V supply voltages and temperatures) - matching original implementation
    uint16_t tempvol = 0;
    
    for (int h = 0; h < chip_count; h++) {
        // Read first word - Internal Temperature 1
        tempvol = data[1 + (h * 9)] * 256 + data[0 + (h * 9)];
        if (tempvol != 0xffff) {
            chip_temperatures[h] = tempvol;  // Store raw temperature value
        }
        
        // Read second word - 5V Supply Voltage
        tempvol = data[3 + (h * 9)] * 256 + data[2 + (h * 9)];
        if (tempvol != 0xffff) {
            // Some chips need byte order reversed (0,3,5,7) due to different endianness
            if (h == 0 || h == 3 || h == 5 || h == 7) {
                chip_5v_supply[h] = rev16(tempvol);  // Reverse byte order for specific chips
            } else {
                chip_5v_supply[h] = tempvol;  // Store as is for other chips
            }
        }
        
        Serial.printf("Chip %d: Temp1=%d, 5V=%d\n", h, chip_temperatures[h], chip_5v_supply[h]);
    }
}

void BatmanBattery::process_temperature_data(uint8_t* data) {
    // Process temperature data
    chip_temperatures[0] = (data[0] << 8) | data[1];
}

uint8_t BatmanBattery::calc_crc(uint8_t* data, uint8_t length) {
    uint16_t crc = 0x0010;  // Initial CRC14 value
    for (int i = 0; i < length; i++) {
        crc = crc14table[(crc >> 6) ^ data[i]] ^ (crc << 8);
        crc &= 0x3FFF;  // Keep only 14 bits
    }
    return (uint8_t)(crc >> 6);  // Return upper 8 bits
}

void BatmanBattery::crc14_bytes(uint8_t len, uint8_t* bytes, uint16_t* crc) {
    // CRC14 calculation for bytes
    for (uint8_t i = 0; i < len; i++) {
        *crc = crc14table[(*crc >> 6) ^ bytes[i]] ^ (*crc << 8);
        *crc &= 0x3FFF;  // Keep only 14 bits
    }
}

void BatmanBattery::crc14_bits(uint8_t len, uint8_t data, uint16_t* crc) {
    // CRC14 calculation for individual bits
    uint8_t topBits = data << (8 - len);
    for (uint8_t i = 0; i < len; i++) {
        uint8_t bit = (topBits & 0x80) ? 1 : 0;
        topBits <<= 1;
        
        if ((*crc & 0x2000) != 0) {  // Check bit 13 (MSB of 14-bit CRC)
            *crc = ((*crc << 1) ^ 0x025B) | bit;
        } else {
            *crc = (*crc << 1) | bit;
        }
        *crc &= 0x3FFF;  // Keep only 14 bits
    }
}

BatmanBattery::CellPosition BatmanBattery::get_cell_hardware_position(int sequential_cell_num) const {
    CellPosition pos = {0, 0, false};
    int cell_count = 0;
    
    for (int chip = 0; chip < chip_count; chip++) {
        for (int cell = 0; cell < 15; cell++) {
            if (cell_voltages[chip][cell] > 1000) { // Cell is present (>1V)
                cell_count++;
                if (cell_count == sequential_cell_num) {
                    pos.chip = chip;
                    pos.register_pos = cell;
                    pos.valid = true;
                    return pos;
                }
            }
        }
    }
    return pos;
}

int BatmanBattery::get_sequential_cell_number(int chip, int register_pos) const {
    int cell_count = 0;
    
    for (int i = 0; i < chip_count; i++) {
        for (int j = 0; j < 15; j++) {
            if (cell_voltages[i][j] > 1000) { // Cell is present (>1V)
                cell_count++;
                if (i == chip && j == register_pos) {
                    return cell_count;
                }
            }
        }
    }
    return 0;
}

bool BatmanBattery::check_spi_communication() {
    // Test SPI communication by trying to read a register
    Serial.println("Attempting to wake up Batman IC...");
    batman_wake_up();
    delay(10);
    batman_idle_wake();
    delay(10);
    
    // Try to read Aux A register
    uint8_t temp_data[2] = {0x4D, 0};
    uint16_t req_data[2];
    
    req_data[0] = 0x4D << 8;
    req_data[1] = (calc_crc(temp_data, 2)) << 8;
    
    gpio_set_level(BMB_CS, 0);
    spi_xfer(req_data[0]);
    spi_xfer(req_data[1]);
    
    // Read a few bytes to test response
    uint16_t response1 = spi_xfer(0x0000);
    uint16_t response2 = spi_xfer(0x0000);
    gpio_set_level(BMB_CS, 1);
    
    Serial.printf("SPI Test - Response1: 0x%04X, Response2: 0x%04X\n", response1, response2);
    
    // Check if we're getting all 0xFF (no response) or all 0x00 (no communication)
    if ((response1 == 0xFFFF && response2 == 0xFFFF) || 
        (response1 == 0x0000 && response2 == 0x0000)) {
        return false;
    }
    return true;
}

void BatmanBattery::reset_communication_timeout() {
    bmb_timeout = false;
} 