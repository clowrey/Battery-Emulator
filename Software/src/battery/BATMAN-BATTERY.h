#ifndef BATMAN_BATTERY_H
#define BATMAN_BATTERY_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include "../datalayer/datalayer.h"
#include "../include.h"
#include "Battery.h"
#include "BATMAN-HTML.h"

#ifdef BATMAN_BATTERY
#define SELECTED_BATTERY_CLASS BatmanBattery
#endif

// Batman IC SPI Configuration
#define BMB_SPI_HOST    SPI3_HOST  // Use VSPI
#define BMB_ENABLE      GPIO_NUM_19
#define BMB_MISO        GPIO_NUM_5
#define BMB_MOSI        GPIO_NUM_17
#define BMB_SCK         GPIO_NUM_16
#define BMB_CS          GPIO_NUM_18

// AS8510 Current Sensor Configuration
#define AS8510_CS_PIN       14
#define AS8510_MOSI_PIN     26
#define AS8510_MISO_PIN     27
#define AS8510_SCK_PIN      25
#define SHUNT_RESISTANCE    0.000025296  // 25296nΩ shunt resistance

class BatmanBattery : public Battery {
 public:
  BatmanBattery();
  virtual ~BatmanBattery();

  virtual void setup(void) override;
  virtual void update_values() override;
  virtual String interface_name() override { return "Tesla Batman IC + AS8510"; }

  // Batman-specific capabilities
  virtual bool supports_manual_balancing() override { return true; }
  virtual bool supports_real_BMS_status() override { return true; }
  
  // HTML renderer for web interface
  virtual BatteryHtmlRenderer& get_status_renderer() override { return renderer; }

  static constexpr const char* Name = "Tesla Model 3 Batman BMS";

 private:
  // HTML renderer for web interface
  BatmanHtmlRenderer renderer;
  
  // Batman IC SPI communication
  spi_device_handle_t spi_dev;
  void init_batman_spi();
  uint16_t spi_xfer(uint16_t data);
  void batman_state_machine();

  // Batman IC commands and data
  void batman_wake_up();
  void batman_idle_wake();
  void batman_get_data(uint8_t reg_id);
  void batman_get_temp_data();
  void batman_write_config();
  void batman_take_snapshot();
  
  // Cell voltage processing
  void update_cell_voltages();
  void update_aux_voltages();
  void update_temperatures();
  void process_balancing();

  // AS8510 current sensor
  void init_as8510();
  void read_current_sensor();
  float calculate_current(uint32_t raw_value);

  // Data storage
  uint16_t cell_voltages[8][15];  // 8 chips x 15 registers
  uint16_t cell_balance_cmd[8];   // Balance control commands
  uint16_t chip_temperatures[8];
  uint16_t chip_5v_supply[8];
  uint16_t chip_voltages[8];
  
  // State management
  uint8_t state_machine_step;
  uint8_t chip_count;
  uint8_t balance_phase;  // 0=measurement, 1=even cells, 2=odd cells
  bool bmb_timeout;
  bool balance_enabled;
  
  // Timing control
  unsigned long last_update_10ms;
  unsigned long last_update_1s;
  unsigned long last_state_machine;
  
  // Cell statistics
  float cell_voltage_max;
  float cell_voltage_min;
  float temperature_max;
  float temperature_min;
  uint16_t cells_present;
  uint16_t cells_balancing;
  
  // Current measurement
  float pack_current_A;
  float pack_power_W;
  
  // Helper functions
  uint8_t calc_crc(uint8_t* data, uint8_t length);
  void crc14_bytes(uint8_t len, uint8_t* bytes, uint16_t* crc);
  void crc14_bits(uint8_t len, uint8_t data, uint16_t* crc);
  uint16_t rev16(uint16_t x) { return ((x & 0xFF00) >> 8) | ((x & 0x00FF) << 8); }
  
  // Error handling and diagnostics
  bool check_spi_communication();
  void reset_communication_timeout();
  
  // Data processing functions
  void process_register_data(uint8_t reg_id, uint8_t* data);
  void process_cell_voltage_data(uint8_t reg_id, uint8_t* data);
  void process_aux_data(uint8_t* data);
  void process_temperature_data(uint8_t* data);
  
  // Calculation functions
  uint16_t calculate_soc();
  void calculate_power_limits();
  void update_datalayer_values();
  
  // Helper functions for cell mapping
  struct CellPosition {
    int chip;
    int register_pos;
    bool valid;
  };
  
  CellPosition get_cell_hardware_position(int sequential_cell_num) const;
  int get_sequential_cell_number(int chip, int register_pos) const;
  
  // Constants
  static const int MAX_PACK_VOLTAGE_DV = 4200;  // 420.0V
  static const int MIN_PACK_VOLTAGE_DV = 2800;  // 280.0V
  static const int MAX_CELL_VOLTAGE_MV = 4250;  // 4.25V
  static const int MIN_CELL_VOLTAGE_MV = 2700;  // 2.7V
  static const int MAX_CELL_DEVIATION_MV = 100; // 100mV
  static const int BALANCE_HYSTERESIS_MV = 20;  // 20mV balance hysteresis
  
  // Timeout constants
  static const unsigned long BMB_TIMEOUT_MS = 5000;  // 5 second timeout
  static const unsigned long CURRENT_SENSOR_TIMEOUT_MS = 2000;  // 2 second timeout
  
  // Cell voltage constants
  static const uint16_t VALID_CELL_VOLTAGE_THRESHOLD_MV = 1000;  // 1V minimum for valid cell
  static const uint16_t INVALID_VOLTAGE_MV = 0;  // Sentinel value for invalid voltage
  static const int16_t INVALID_TEMPERATURE_dC = 0;  // Sentinel value for invalid temperature
  
  static const unsigned long STATE_MACHINE_INTERVAL = 100;  // 100ms
  static const unsigned long UPDATE_10MS_INTERVAL = 10;
  static const unsigned long UPDATE_1S_INTERVAL = 1000;
};

#endif // BATMAN_BATTERY_H 