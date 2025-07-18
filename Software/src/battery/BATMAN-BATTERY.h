#ifndef BATMAN_BATTERY_H
#define BATMAN_BATTERY_H
#include "../datalayer/datalayer.h"
#include "../include.h"
#include "CanBattery.h"
#include "BATMAN-HTML.h"

#ifdef BATMAN_BATTERY
// Include the core SPI functionality from Batman IC
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <cmath>

// ESP32 SPI Configuration for Batman IC
#define BMB_SPI_HOST    SPI2_HOST    
#define BMB_ENABLE      GPIO_NUM_19
#define BMB_MISO        GPIO_NUM_5   
#define BMB_MOSI        GPIO_NUM_17  
#define BMB_SCK         GPIO_NUM_16  
#define BMB_CS          GPIO_NUM_18  

// Batman IC Commands
#define BATMAN_WAKEUP   0x2AD4
#define BATMAN_UNMUTE   0x21F2
#define BATMAN_MUTE     0x20DD
#define BATMAN_SNAP     0x2BFB
#define BATMAN_READ_A   0x4700
#define BATMAN_READ_B   0x4800
#define BATMAN_READ_C   0x4900
#define BATMAN_READ_D   0x4A00
#define BATMAN_READ_E   0x4B00
#define BATMAN_READ_F   0x4C00
#define BATMAN_READ_AUX_A 0x4D00
#define BATMAN_READ_AUX_B 0x4E00
#define BATMAN_READ_STATUS 0x4F00
#define BATMAN_READ_CFG 0x5000

// Helper function to reverse 16-bit value
static inline uint16_t rev16(uint16_t x) {
    return ((x & 0xFF00) >> 8) | ((x & 0x00FF) << 8);
}

#endif

#ifdef BATMAN_BATTERY
#define SELECTED_BATTERY_CLASS BatmanBattery
#endif

class BatmanBattery : public CanBattery {
 public:
  // Use this constructor for the second battery.
  BatmanBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr, CAN_Interface targetCan) : CanBattery(targetCan), htmlRenderer(*this) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
  }

  // Use the default constructor to create the first or single battery.
  BatmanBattery() : htmlRenderer(*this) {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
  }

  static constexpr const char* Name = "Tesla Model 3 Batman IC Battery Management";

  virtual void setup();
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);

  // Override interface_name to identify this as a Batman battery for web server
  String interface_name() override { return "Batman CAN"; }

  bool supports_set_fake_voltage() { return false; }  // Real cell monitoring, not fake values
  void set_fake_voltage(float val) { }  // Not supported for real battery

  // Balancing support for settings webpage
  bool supports_manual_balancing() { return true; }

  // HTML renderer for web interface
  BatteryHtmlRenderer& get_status_renderer() { return htmlRenderer; }

#ifdef BATMAN_BATTERY
  // Batman IC specific methods
  void BatStart();
  void loop();
  void StateMachine();
  void IdleWake();
  void GetData(uint8_t ReqID);
  void WriteCfg();
  void GetTempData();
  void WakeUP();
  void Generic_Send_Once(uint16_t Command[], uint8_t len);
  void upDateCellVolts();
  void upDateAuxVolts();
  void upDateTemps();
  void updateIndividualCellVoltageParameters(void);
  uint8_t calcCRC(uint8_t *inData, uint8_t Length);
  void crc14_bytes(uint8_t len_B, uint8_t *bytes, uint16_t *crcP);
  void crc14_bits(uint8_t len_b, uint8_t inB, uint16_t *crcP);
  uint16_t spi_xfer(spi_host_device_t host, uint16_t data);
  bool checkSPIConnection();

  // Helper methods for cell position mapping
  struct CellPosition {
      int chip;
      int register_pos;
      bool valid;
  };
  
  CellPosition getCellHardwarePosition(int sequentialCellNum) const;
  int getSequentialCellNumber(int chip, int register_pos) const;
  void printHardwareMapping() const;
  
  // Balancing information structure
  struct BalancingInfo {
      int totalCells;
      int balancingCells;
      int balancingCellNumbers[108]; // Array to store cell numbers being balanced
  };
  
  BalancingInfo getBalancingInfo() const;
  
  // Get voltage from specific chip and register position
  uint16_t getVoltage(int chip, int register_pos) const {
      if (chip >= 0 && chip < 8 && register_pos >= 0 && register_pos < 15) {
          return Voltage[chip][register_pos];
      }
      return 0;
  }

  // Getter methods for monitoring
  float getMinVoltage() const { return CellVMin; }
  float getMaxVoltage() const { return CellVMax; }
  int getMinCell() const;
  int getMaxCell() const;
  
  // Balancing control methods
  bool isBalancingEnabled() const { return BalancingEnabled; }
  void setBalancingEnabled(bool enabled) { 
    if (BalancingEnabled != enabled) {
      BalancingEnabled = enabled; 
      BalancingStateChanged = true;  // Flag that we need to recalculate balancing commands
    }
  }
  void toggleBalancing() { 
    BalancingEnabled = !BalancingEnabled; 
    BalancingStateChanged = true;  // Flag that we need to recalculate balancing commands
  }
  
  // Balancing status methods (read-only)
  bool getBalancingStatus() const { return BalanceFlag; }  // Shows if cells currently need balancing
  
  // BMB status methods
  bool getBmbTimeout() const { return BmbTimeout; }
  uint8_t getActiveBmbCount() const { return ActualBmbCount; }
  uint8_t getBalancePhase() const { return BalancePhase; }

private:
  // BMB Connectivity Monitoring
  bool validateBmbResponse(uint8_t chipIndex, uint8_t reqID);
  void updateBmbConnectivity();
  void markBmbDataAsStale(uint8_t chipIndex);
  int getCellParameterIndex(uint8_t chipIndex, uint8_t regIndex);
#endif

 private:
  DATALAYER_BATTERY_TYPE* datalayer_battery;
  // If not null, this battery decides when the contactor can be closed and writes the value here.
  bool* allows_contactor_closing;
  
  // HTML renderer for web interface
  BatmanHtmlRenderer htmlRenderer;

  static const int MAX_CELL_DEVIATION_MV = 9999;

  unsigned long previousMillis10 = 0;   // will store last time a 10ms CAN Message was send
  unsigned long previousMillis100 = 0;  // will store last time a 100ms CAN Message was send
  unsigned long previousMillis10s = 0;  // will store last time a 1s CAN Message was send

#ifdef BATMAN_BATTERY
  // Batman IC implementation variables
  spi_device_handle_t spi_dev;
  uint8_t ChipNum;
  uint16_t Voltage[8][15];
  uint16_t CellBalCmd[8];
  uint16_t Temps[8];
  uint16_t Temp1[8];
  uint16_t Temp2[8];
  uint16_t Volts5v[8];
  uint16_t ChipV[8];
  uint16_t Cfg[8][2];
  float CellVMax;
  float CellVMin;
  float TempMax;
  float TempMin;
  bool BalanceFlag;                // Status: Shows if cells currently need balancing (read-only)
  bool BalancingEnabled;          // Control: Enable/disable balancing phase cycling
  bool BalancingStateChanged;     // Flag: Indicates balancing state has changed and commands need recalculation
  bool BmbTimeout;
  uint16_t LoopState;
  uint16_t LoopRanCnt;
  uint8_t WakeCnt;
  uint8_t WaitCnt;
  uint16_t IdleCnt;
  uint16_t SendDelay;
  uint32_t lasttime;
  uint8_t BalancePhase;  // 0=measurement only (no balancing), 1=even cells, 2=odd cells
  uint16_t LastCellBalancing;  // Preserve balancing count across phases
  float Cell1start;
  float Cell2start;
  
  // BMB Connectivity Monitoring
  bool BmbConnected[8];           // Track which BMBs are currently responding
  unsigned long LastBmbResponse[8]; // Timestamp of last valid response from each BMB
  uint8_t ActualBmbCount;         // Actual number of responding BMBs
  static const unsigned long BMB_TIMEOUT_MS = 10000; // 10 second timeout for BMB responses
  
  // Debug control for detailed register analysis
  static bool _registerDebugEnabled;
#endif

  CAN_frame TEST = {.FD = false,
                    .ext_ID = false,
                    .DLC = 8,
                    .ID = 0x123,
                    .data = {0x10, 0x64, 0x00, 0xB0, 0x00, 0x1E, 0x00, 0x8F}};
};

#endif