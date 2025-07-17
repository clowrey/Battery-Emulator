// AS8510.h
#ifndef AS8510_H
#define AS8510_H

#include <Arduino.h>
#include <SPI.h>
#include <cstring>
#include <stdint.h>
#include <math.h>

// Register addresses for AS8510
#define AS8510_DREG_I1        0x00  // Current data registers
#define AS8510_DREG_I2        0x01
#define AS8510_DREG_V1        0x02  // Voltage data registers
#define AS8510_DREG_V2        0x03
#define AS8510_STATUS_REG     0x04  // Read-only status register
#define AS8510_DEC_REG_R1_I   0x05  // Control register 1
#define AS8510_DEC_REG_R2_I   0x06  // Control register 2
#define AS8510_FIR_CTL_REG_I  0x07
#define AS8510_CLK_REG        0x08  // Clock control register
#define AS8510_RESET_REG      0x09  // Soft-reset control
#define AS8510_MOD_CTL_REG    0x0A  // Mode control register
#define AS8510_MOD_TA_REG1    0x0B  // Timing control for SBM1/SBM2
#define AS8510_MOD_TA_REG2    0x0C
#define AS8510_MOD_ITH_REG1   0x0D  // Threshold control for SBM1/SBM2
#define AS8510_MOD_ITH_REG2   0x0E
#define AS8510_MOD_TMC_REG1   0x0F  // Configures number of ADC samples to drop
#define AS8510_MOD_TMC_REG2   0x10
#define AS8510_NOM_ITH_REG1   0x11  // Threshold control for NOM2
#define AS8510_NOM_ITH_REG2   0x12
#define AS8510_PGA_CTL_REG    0x13  // Gain control
#define AS8510_PD_CTL_REG_1   0x14  // Power control registers
#define AS8510_PD_CTL_REG_2   0x15
#define AS8510_PD_CTL_REG_3   0x16
#define AS8510_ACH_CTL_REG    0x17  // Analog channel selection
#define AS8510_ISC_CTL_REG    0x18  // Current source setting register
#define AS8510_OTP_EN_REG     0x19  // Reserved
#define AS8510_STATUS_REG_2   0x44  // Data saturation flags
#define AS8510_DEC_R1_V       0x45  // Voltage control registers
#define AS8510_DEC_R2_V       0x46
#define AS8510_FIR_CTL_REG_V  0x47

// Constants from Rust library
#define CALIBRATION           0x3670
#define ADDR_CURRENT          0x00
#define ADDR_VOLTAGE          0x02
#define ADDR_STATUS           0x04

// Status register bits (0x04)
#define STATUS_CURRENT_CHANNEL_UPDATED  (1 << 2)
#define STATUS_VOLTAGE_CHANNEL_UPDATED  (1 << 1)
#define STATUS_TEMPERATURE_READY        (1 << 0)
#define STATUS_OVERCURRENT_FAULT        (1 << 7)
#define STATUS_OVERVOLTAGE_FAULT        (1 << 6)
#define STATUS_UNDERVOLTAGE_FAULT       (1 << 5)
#define STATUS_OVERTEMPERATURE_FAULT    (1 << 4)
#define STATUS_COMMUNICATION_ERROR      (1 << 3)

// Status register 2 bits (0x44) - Data saturation flags
#define STATUS2_CURRENT_SATURATION      (1 << 7)
#define STATUS2_VOLTAGE_SATURATION      (1 << 6)
#define STATUS2_TEMP_SATURATION         (1 << 5)
#define STATUS2_PGA_SATURATION          (1 << 4)
#define STATUS2_ADC_OVERFLOW            (1 << 3)
#define STATUS2_ADC_UNDERFLOW           (1 << 2)
#define STATUS2_CLOCK_ERROR             (1 << 1)
#define STATUS2_POWER_SUPPLY_ERROR      (1 << 0)

// Mode control bits
#define MOD_CTL_START         0x01

// ACH_CTL_REG (0x17) - Analog channel selection register
// D[7:6] - Channel selection bits
#define ACH_CTL_VOLTAGE_CHANNEL         0x00  // 00 = Voltage channel
#define ACH_CTL_EXTERNAL_TEMP_ETR       0x40  // 01 = External temperature channel ETR  
#define ACH_CTL_EXTERNAL_TEMP_ETS       0x80  // 10 = External temperature channel ETS
#define ACH_CTL_INTERNAL_TEMP_CHANNEL   0xC0  // 11 = Internal temperature channel

// D[4] - Internal current source enable
#define ACH_CTL_CURRENT_SOURCE_ENABLE   0x10  // Enable current source to selected channel

// ISC_CTL_REG (0x18) - Current source setting register  
// D[7:3] - Current source magnitude selection (5-bit pattern in bit positions 7:3)
#define ISC_CTL_0UA                     0x00  // 00000 = 0μA (default)
#define ISC_CTL_8_5UA                   0x08  // 00001 = 8.5μA (00001000)
#define ISC_CTL_17UA                    0x10  // 00010 = 17μA (00010000)  
#define ISC_CTL_34_5UA                  0x20  // 00100 = 34.5μA (00100000)
#define ISC_CTL_68UA                    0x40  // 01000 = 68μA (01000000)
#define ISC_CTL_135UA                   0x80  // 10000 = 135μA (10000000)
#define ISC_CTL_270UA                   0xF8  // 11111 = 270μA (11111000)

// Gain enumeration (from Rust)
enum class Gain {
    Gain1 = 1,
    Gain5 = 5,      // Added: Required for internal temperature sensor
    Gain25 = 25,
    Gain40 = 40,
    Gain100 = 100
};

class AS8510 {
private:
  SPIClass* _spi;
    uint8_t _csPin;
    uint8_t _mosiPin;
    uint8_t _misoPin;
    uint8_t _sckPin;
  SPISettings _spiSettings;
    
    bool _isInitialized;
    bool _verboseLogging;
    float _shuntResistance;
    uint8_t _data[5];
    Gain _currentGain;
    Gain _voltageGain;
    
    // Private helper methods
    void readRegisters(uint8_t reg, uint8_t* data, uint8_t len);
    uint8_t calculateReg0x13();
    uint8_t calculateReg0x15();
    bool isCurrentChannelUpdated();
    bool isVoltageChannelUpdated();
  
public:
    AS8510(uint8_t csPin, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, 
           Gain currentGain = Gain::Gain100, Gain voltageGain = Gain::Gain25);
  
  bool begin();
    bool isInitialized() { return _isInitialized; }
    
    // Current measurement
    float getCurrent();
    float getVoltage();
    float getBatteryVoltage(); // Measures shunt voltage using voltage channel with current gain
    
    // Temperature measurement
    float getExternalTemperature(); // NTC thermistor temperature
    float getInternalTemperature(); // Internal IC temperature
    
    // Status and diagnostics
    uint8_t getStatus();
    void printStatus();
    
    // Error code reading and diagnostics
    uint8_t getErrorCodes();
    uint8_t getSaturationFlags();
    void printErrorCodes();
    void printSaturationFlags();
    void printAllDiagnostics();
  
  // Utility methods
  float getShuntResistance() { return _shuntResistance; }
  void setShuntResistance(float resistance) { _shuntResistance = resistance; }
  
    // Verbose logging control
    void setVerboseLogging(bool verbose) { _verboseLogging = verbose; }
    bool getVerboseLogging() { return _verboseLogging; }
  
    // Register access for debugging
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    
    // Explicit start method
    void startDevice();
    
    // Compatibility methods for old interface
    bool isDevicePresent() { return _isInitialized; }
    bool isAwake() { return _isInitialized; }
    bool isDataReady() { return isCurrentChannelUpdated() || isVoltageChannelUpdated(); }
    int16_t readRawADC(uint8_t channel = 1);
    float readVoltage(uint8_t channel = 1) { return getVoltage(); }
    float readCurrent(uint8_t channel = 1) { return getCurrent(); }
};

// Implementation
AS8510::AS8510(uint8_t csPin, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, 
               Gain currentGain, Gain voltageGain)
    : _csPin(csPin), _mosiPin(mosiPin), _misoPin(misoPin), _sckPin(sckPin),
      _currentGain(currentGain), _voltageGain(voltageGain),
      _isInitialized(false), _verboseLogging(false), _shuntResistance(0.000025296),
      _spiSettings(1000000, MSBFIRST, SPI_MODE1) {
    
    // Initialize data buffer
    memset(_data, 0, sizeof(_data));
    
    // Validate voltage gain (expanded from Rust assertion to include Gain5 for temperature)
    if (_voltageGain != Gain::Gain5 && _voltageGain != Gain::Gain25 && _voltageGain != Gain::Gain40) {
        Serial.println("AS8510: WARNING - Voltage gain must be Gain5, Gain25, or Gain40");
        _voltageGain = Gain::Gain25;  // Default to safe value
    }
}

bool AS8510::begin() {
    Serial.println("AS8510: Starting initialization with Rust-based sequence...");
    
    // Configure SPI pins
  pinMode(_csPin, OUTPUT);
    pinMode(_mosiPin, OUTPUT);
    pinMode(_misoPin, INPUT);
    pinMode(_sckPin, OUTPUT);
    
  digitalWrite(_csPin, HIGH);
  
    // Initialize VSPI (no LCD conflict - LCD disabled)
    _spi = new SPIClass(VSPI);
    _spi->begin(_sckPin, _misoPin, _mosiPin, _csPin);
    
    Serial.println("AS8510: VSPI initialized, starting register configuration...");
    
    // Exact initialization sequence from Rust library
    writeRegister(AS8510_DEC_REG_R1_I, 0b01000101);    // 0x45
    writeRegister(AS8510_DEC_REG_R2_I, 0b11000101);    // 0xC5
    writeRegister(AS8510_FIR_CTL_REG_I, 0b00000100);   // 0x04
    writeRegister(AS8510_CLK_REG, 0b00100000);         // 0x20
    writeRegister(AS8510_MOD_CTL_REG, 0);              // 0x00 - Clear mode control
    writeRegister(AS8510_MOD_TA_REG1, 0b10000000);     // 0x80
    writeRegister(AS8510_MOD_TA_REG2, 0);              // 0x00
    writeRegister(AS8510_MOD_ITH_REG1, 0b01010000);    // 0x50
    writeRegister(AS8510_MOD_ITH_REG2, 0b11001111);    // 0xCF
    writeRegister(AS8510_MOD_TMC_REG1, 0b11110011);    // 0xF3
    writeRegister(AS8510_MOD_TMC_REG2, 0b11111000);    // 0xF8
    writeRegister(AS8510_NOM_ITH_REG1, 0);             // 0x00
    writeRegister(AS8510_NOM_ITH_REG2, 0);             // 0x00
    writeRegister(AS8510_PGA_CTL_REG, calculateReg0x13());     // Gain control
    writeRegister(AS8510_PD_CTL_REG_1, 0b11001111);    // 0xCF
    writeRegister(AS8510_PD_CTL_REG_2, calculateReg0x15());   // Power control
    writeRegister(AS8510_PD_CTL_REG_3, 0b11111000);    // 0xF8
    writeRegister(AS8510_ACH_CTL_REG, 0);              // 0x00
    writeRegister(AS8510_ISC_CTL_REG, 0);              // 0x00
    writeRegister(AS8510_OTP_EN_REG, 0);               // 0x00
    
    Serial.println("AS8510: Register configuration complete, starting device...");
    
    // Start read - CRITICAL: Only START bit (0x01), no CONT bit!
    writeRegister(AS8510_MOD_CTL_REG, MOD_CTL_START);
    delay(100);
    
    // Verify START bit is set
    uint8_t modCtl = readRegister(AS8510_MOD_CTL_REG);
    Serial.printf("AS8510: Mode Control after start: 0x%02X\n", modCtl);
    
    if (modCtl & MOD_CTL_START) {
        Serial.println("AS8510: SUCCESS - Device started successfully!");
        _isInitialized = true;
  } else {
        Serial.println("AS8510: FAILED - START bit not set");
        _isInitialized = false;
    }
    
    return _isInitialized;
}

uint8_t AS8510::calculateReg0x13() {
    // AS8510 DATASHEET IMPLEMENTATION - PGA_CTL_REG bit patterns
    // D[7:6] = Current channel gain, D[5:4] = Voltage channel gain
    // 00=Gain5, 01=Gain25, 10=Gain40, 11=Gain100
    
    uint8_t currentGainBits = 0;
    switch (_currentGain) {
        case Gain::Gain1:   currentGainBits = 0b00; break; // 0 - Keep for compatibility
        case Gain::Gain5:   currentGainBits = 0b00; break; // 0 - Datasheet: 00=Gain5
        case Gain::Gain25:  currentGainBits = 0b01; break; // 1 - Datasheet: 01=Gain25
        case Gain::Gain40:  currentGainBits = 0b10; break; // 2 - Datasheet: 10=Gain40
        case Gain::Gain100: currentGainBits = 0b11; break; // 3 - Datasheet: 11=Gain100
    }
    
    uint8_t voltageGainBits = 0;
    switch (_voltageGain) {
        case Gain::Gain1:   voltageGainBits = 0b00; break; // 0 - Keep for compatibility  
        case Gain::Gain5:   voltageGainBits = 0b00; break; // 0 - Datasheet: 00=Gain5
        case Gain::Gain25:  voltageGainBits = 0b01; break; // 1 - Datasheet: 01=Gain25
        case Gain::Gain40:  voltageGainBits = 0b10; break; // 2 - Datasheet: 10=Gain40
        case Gain::Gain100: voltageGainBits = 0b11; break; // 3 - Datasheet: 11=Gain100
    }
    
    // PGA_CTL_REG format: [D7:D6]=Current gain, [D5:D4]=Voltage gain, [D3:D0]=Reserved
    uint8_t result = (currentGainBits << 6) | (voltageGainBits << 4);
    
    return result;
}

uint8_t AS8510::calculateReg0x15() {
    // Power control register calculation from Rust
    uint8_t reg = 0xF0;
    
    if (_currentGain == Gain::Gain1) {
        reg |= 0x0F;
  } else {
        reg |= 0x02;
    }
    
    if (_voltageGain != Gain::Gain1) {
        reg |= 0x01;
    }
    
    return reg;
}

void AS8510::writeRegister(uint8_t reg, uint8_t value) {
    if (reg > 0x47) {
        Serial.printf("AS8510: ERROR - Illegal register address: 0x%02X\n", reg);
        return;
    }
    
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    
    _spi->transfer(reg);     // Address
    _spi->transfer(value);   // Data
    
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    
    delayMicroseconds(10);
    
    // Critical register write logging commented out for cleaner output
    // if (reg == AS8510_MOD_CTL_REG || reg == AS8510_PGA_CTL_REG || reg == AS8510_PD_CTL_REG_2) {
    //     Serial.printf("AS8510: REG[0x%02X] = 0x%02X\n", reg, value);
    // }
}

uint8_t AS8510::readRegister(uint8_t reg) {
    if (reg > 0x47) {
        Serial.printf("AS8510: ERROR - Illegal register address: 0x%02X\n", reg);
        return 0;
    }
  
    _spi->beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  
    _spi->transfer(reg | 0x80);  // Read command (set MSB)
    uint8_t value = _spi->transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    
    delayMicroseconds(10);
  
    // Status register logging commented out for cleaner output
    // if (reg == AS8510_STATUS_REG) {
    //     Serial.printf("AS8510: Status = 0x%02X %s\n", value, (value & 0xC0) ? "[DATA_READY]" : "[WAITING]");
    // }
  return value;
}

void AS8510::readRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    if (reg + len > 0x48) {
        Serial.printf("AS8510: ERROR - Read beyond register range\n");
        return;
    }
    
    _spi->beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  
    _spi->transfer(reg | 0x80);  // Read command
    for (uint8_t i = 0; i < len; i++) {
        data[i] = _spi->transfer(0x00);
  }
  
  digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    
    delayMicroseconds(10);
}

float AS8510::getCurrent() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return 0.0;
    }
    
    // Check if current channel is updated
    if (!isCurrentChannelUpdated()) {
        Serial.println("AS8510: Current channel not ready");
        return 0.0;
    }
    
    // HIGH-PRECISION SAMPLING: 1kHz sampling rate, 1000 samples averaged
    const int numSamples = 1000; // 1000 samples = 1 second of data at 1kHz
    int64_t adcSum = 0; // Use 64-bit to prevent overflow with 1000 samples
    uint16_t minRawADC = 65535;
    uint16_t maxRawADC = 0;
    uint16_t firstRawADC = 0;
    uint16_t lastRawADC = 0;
    
    if (_verboseLogging) {
        Serial.println("┌─── Sampling 1000 readings ───");
        Serial.printf("│ Rate: 1kHz (1ms intervals)\n");
        Serial.printf("│ Progress: ");
    }
    
    for (int i = 0; i < numSamples; i++) {
        // Read current data (2 bytes)
        readRegisters(ADDR_CURRENT, _data, 2);
        
        uint16_t rawADC = (_data[0] << 8) | _data[1]; // Unsigned 16-bit ADC reading
        int32_t adc = (int32_t)rawADC - 32768; // Center around zero
        
        adcSum += adc;
        
        // Track statistics
        if (i == 0) firstRawADC = rawADC;
        if (i == numSamples - 1) lastRawADC = rawADC;
        if (rawADC < minRawADC) minRawADC = rawADC;
        if (rawADC > maxRawADC) maxRawADC = rawADC;
        
        // Show progress every 100 samples
        if (_verboseLogging && i % 100 == 0) {
            Serial.printf("%d%% ", (i * 100) / numSamples);
        }
        
        delay(1); // 1ms delay = 1kHz sampling rate
    }
    
    if (_verboseLogging) {
        Serial.println("100%");
        Serial.println("│ Sampling complete!");
    }
    
    // Calculate averaged values
    int32_t avgADC = (int32_t)(adcSum / numSamples);
    float avgCurrent = (float)avgADC * 0.01523; // Apply scale factor to average (calibrated for -3A actual)
    int32_t adcRange = (int32_t)maxRawADC - (int32_t)minRawADC;
    
    // Show comprehensive statistics
    if (_verboseLogging) {
        Serial.println("├─── Statistics ───────────────");
        Serial.printf("│ Samples:  %4d readings\n", numSamples);
        Serial.printf("│ Min ADC:  0x%04X (%5u)\n", minRawADC, minRawADC);
        Serial.printf("│ Max ADC:  0x%04X (%5u)\n", maxRawADC, maxRawADC);
        Serial.printf("│ Range:    %5d counts\n", adcRange);
        Serial.printf("│ Avg ADC:  %6d counts\n", avgADC);
        Serial.printf("│ Current:  %7.4f A\n", avgCurrent);
        Serial.println("└─────────────────────────────");
    }
    
    return avgCurrent;
}

float AS8510::getVoltage() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return 0.0;
    }
    
    // Check if voltage channel is updated
    if (!isVoltageChannelUpdated()) {
        Serial.println("AS8510: Voltage channel not ready");
        return 0.0;
    }
    
    // Read voltage data (2 bytes)
    readRegisters(ADDR_VOLTAGE, _data, 2);
    
    // EXACT RUST IMPLEMENTATION (same as current but with voltage gain):
    int16_t rawADC = (int16_t)(_data[0] << 8 | _data[1]); // Big-endian conversion
    
    // Check for saturation (0xFFFF indicates potential problem)
    if (rawADC == -1) {
        Serial.println("AS8510: WARNING - Voltage channel saturated (0xFFFF)");
        return 0.0; // Return 0 for saturated readings
    }
    
    int32_t adc = (int32_t)rawADC + 0x8000; // Add 0x8000 (convert signed to unsigned range)
    
    // EXACT RUST FORMULA (but with voltage gain):
    float voltage = ((float)(adc * CALIBRATION) / (float)((int32_t)_voltageGain) / 2000.0) * 0.1;
    
        Serial.printf("AS8510: Voltage: %.3f V\n", voltage);
    
  return voltage;
}

float AS8510::getBatteryVoltage() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return 0.0;
    }
    
    // Check if voltage channel is updated
    if (!isVoltageChannelUpdated()) {
        Serial.println("AS8510: Voltage channel not ready for battery voltage measurement");
        return 0.0;
    }
    
    // Store original settings
    uint8_t originalACH = readRegister(AS8510_ACH_CTL_REG);
    uint8_t originalPGA = readRegister(AS8510_PGA_CTL_REG);
    Gain originalVoltageGain = _voltageGain;
    
    // Set voltage channel to use current gain settings for shunt measurement
    if (_verboseLogging) {
        Serial.println("AS8510: Battery voltage measurement - using voltage channel with current gain");
    }
    _voltageGain = _currentGain;
    uint8_t tempPGA = calculateReg0x13();
    writeRegister(AS8510_PGA_CTL_REG, tempPGA);
    
    // Ensure we're on the battery voltage channel 
    // From datasheet: D[7:6] = 00 = Voltage channel, D[4] = 0 = No current source
    writeRegister(AS8510_ACH_CTL_REG, ACH_CTL_VOLTAGE_CHANNEL);
    delay(20); // Allow channel switching and PGA settling
    
    // HIGH-PRECISION SAMPLING: 1kHz sampling rate, 1000 samples averaged (same as current)
    const int numSamples = 1000; // 1000 samples = 1 second of data at 1kHz
    int64_t adcSum = 0; // Use 64-bit to prevent overflow with 1000 samples
    uint16_t minRawADC = 65535;
    uint16_t maxRawADC = 0;
    uint16_t firstRawADC = 0;
    uint16_t lastRawADC = 0;
    
    if (_verboseLogging) {
        Serial.println("┌─── Sampling 1000 readings (Battery Voltage Channel) ───");
        Serial.printf("│ Rate: 1kHz (1ms intervals)\n");
        Serial.printf("│ Progress: ");
    }
    
    for (int i = 0; i < numSamples; i++) {
        // Read voltage data using voltage channel (2 bytes)
        readRegisters(ADDR_VOLTAGE, _data, 2);
        
        uint16_t rawADC = (_data[0] << 8) | _data[1]; // Unsigned 16-bit ADC reading
        int32_t adc = (int32_t)rawADC - 32768; // Center around zero
        
        adcSum += adc;
        
        // Track statistics
        if (i == 0) firstRawADC = rawADC;
        if (i == numSamples - 1) lastRawADC = rawADC;
        if (rawADC < minRawADC) minRawADC = rawADC;
        if (rawADC > maxRawADC) maxRawADC = rawADC;
        
        // Show progress every 100 samples
        if (_verboseLogging && i % 100 == 0) {
            Serial.printf("%d%% ", (i * 100) / numSamples);
        }
        
        delay(1); // 1ms delay = 1kHz sampling rate
    }
    
    if (_verboseLogging) {
        Serial.println("100%");
        Serial.println("│ Sampling complete!");
    }
    
    // Calculate averaged values
    int32_t avgADC = (int32_t)(adcSum / numSamples);
    
    // Calculate shunt voltage using current gain (same calculation as voltage but with current gain)
    uint32_t adc_unsigned = (uint32_t)((int32_t)avgADC + 32768); // Convert back to unsigned
    float shuntVoltage = ((float)(adc_unsigned * CALIBRATION) / (float)((int32_t)_currentGain) / 2000.0) * 0.1;
    
    // Convert shunt voltage to current using Ohm's law (I = V/R)
    float calculatedCurrent = shuntVoltage / _shuntResistance;
    
    int32_t adcRange = (int32_t)maxRawADC - (int32_t)minRawADC;
    
    // Show comprehensive statistics
    if (_verboseLogging) {
        Serial.println("├─── Statistics (Battery Voltage Channel) ───");
        Serial.printf("│ Samples:       %4d readings\n", numSamples);
        Serial.printf("│ Min ADC:       0x%04X (%5u)\n", minRawADC, minRawADC);
        Serial.printf("│ Max ADC:       0x%04X (%5u)\n", maxRawADC, maxRawADC);
        Serial.printf("│ Range:         %5d counts\n", adcRange);
        Serial.printf("│ Avg ADC:       %6d counts\n", avgADC);
        Serial.printf("│ Shunt Voltage: %7.6f V\n", shuntVoltage);
        Serial.printf("│ Calculated I:  %7.4f A\n", calculatedCurrent);
        Serial.printf("│ Shunt R:       %.9f Ω\n", _shuntResistance);
        Serial.println("└─────────────────────────────────────────");
    }
    
    // Restore original settings
    _voltageGain = originalVoltageGain;
    writeRegister(AS8510_PGA_CTL_REG, originalPGA);
    writeRegister(AS8510_ACH_CTL_REG, originalACH);
    
    return calculatedCurrent;
}

float AS8510::getExternalTemperature() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return -999.0;
    }
    
    // SIMPLIFIED EXTERNAL TEMPERATURE MEASUREMENT - Same as internal temp but different MUX
    // Based on datasheet, external temperature should be as simple as internal temperature
    // Just use different MUX setting (0x80 instead of 0xC0)
    
    uint8_t originalACH = readRegister(AS8510_ACH_CTL_REG);
    uint8_t originalPGA = readRegister(AS8510_PGA_CTL_REG);
    uint8_t originalISC = readRegister(AS8510_ISC_CTL_REG);
    Gain originalVoltageGain = _voltageGain;
    
    Serial.println("AS8510: External temperature measurement using simplified MUX approach (gain 5)");
    
    // Step 1: Configure current source magnitude for external temperature measurement
    Serial.printf("AS8510: Setting current source to 68μA (ISC_CTL=0x%02X)\n", ISC_CTL_68UA);
    writeRegister(AS8510_ISC_CTL_REG, ISC_CTL_68UA);
    delay(10); // Allow current source to settle
    
    // Step 2: Configure PGA for gain 5 (same as internal temp sensor)
    _voltageGain = Gain::Gain5;
    uint8_t tempPGA = calculateReg0x13();
    
    Serial.printf("AS8510: Temporarily setting PGA to gain 5 for external temp (PGA=0x%02X)\n", tempPGA);
    writeRegister(AS8510_PGA_CTL_REG, tempPGA);
    delay(50); // Allow PGA to settle (datasheet: first sample after gain change is invalid)
    
    // Step 3: Configure MUX for external temperature sensor
    // From datasheet: D[7:6] = 10 = External temperature channel ETS
    // D[4] = 1 = Enable internal current source (required for external temp)
    uint8_t externalTempMux = ACH_CTL_EXTERNAL_TEMP_ETS | ACH_CTL_CURRENT_SOURCE_ENABLE;
    
    Serial.printf("AS8510: Configuring external temperature sensor ETS with current source (ACH_CTL=0x%02X)\n", externalTempMux);
    writeRegister(AS8510_ACH_CTL_REG, externalTempMux);
    delay(10);  // Allow MUX to settle
    
    // Step 4: Wait for voltage channel to be ready and discard first invalid sample
    int retries = 20;
    while (!isVoltageChannelUpdated() && retries > 0) {
        delay(5);
        retries--;
    }
    
    if (retries > 0) {
        // Discard first sample (invalid after gain change)
        readRegisters(ADDR_VOLTAGE, _data, 2);
        delay(10);
        
        // Wait for second (valid) sample
        retries = 20;
        while (!isVoltageChannelUpdated() && retries > 0) {
            delay(5);
            retries--;
        }
    }
    
    if (retries == 0) {
        Serial.println("AS8510: External temp measurement timeout");
        // Restore original settings
        _voltageGain = originalVoltageGain;
        writeRegister(AS8510_PGA_CTL_REG, originalPGA);
        writeRegister(AS8510_ACH_CTL_REG, originalACH);
        writeRegister(AS8510_ISC_CTL_REG, originalISC);
        return -999.0;
    }
    
    // Step 5: Read external temperature sensor data (at gain 5)
    readRegisters(ADDR_VOLTAGE, _data, 2);
    uint16_t tempADC = (_data[0] << 8) | _data[1];
    
    // Step 6: Restore original settings
    _voltageGain = originalVoltageGain;
    writeRegister(AS8510_PGA_CTL_REG, originalPGA);
    writeRegister(AS8510_ACH_CTL_REG, originalACH);
    writeRegister(AS8510_ISC_CTL_REG, originalISC);
    delay(10); // Allow restoration to settle
    
    Serial.printf("AS8510: External temp ADC at gain 5: 0x%04X (%u)\n", tempADC, tempADC);
    
    // Step 7: Check for no sensor connected (saturated reading)
    if (tempADC >= 65535) {
        Serial.printf("AS8510: No external temperature sensor detected (ADC saturated: 0x%04X)\n", tempADC);
        return -999.0;  // No sensor connected
    }
    
    // Step 8: External NTC Temperature Conversion
    // For external NTC thermistor, we need to convert ADC to resistance first
    // Then use NTC equation to convert resistance to temperature
    
    // Convert ADC to voltage (same as internal temp calculation)
    // Temperature sensor slope: 27 Digits/°C at gain 5 (if it were internal sensor)
    // But for external NTC, we need to convert to actual resistance
    
    // For now, use similar calculation as internal but with different calibration
    // This should be calibrated based on the specific external NTC thermistor used
    float tempSlopeDigitsPerDegC = 27.0; // Same slope as internal (to be calibrated)
    float tempAt25C_Digits = 32768.0;    // Mid-range ADC value at 25°C (to be calibrated)
    float referenceTemp = 25.0;          // Reference temperature
    
    // Temperature calculation: T = T_ref + (ADC - ADC_ref) / slope
    float temp_C = referenceTemp + ((float)tempADC - tempAt25C_Digits) / tempSlopeDigitsPerDegC;
    
    Serial.printf("AS8510: External temp: ADC=%u (gain 5), T=%.1f°C (needs calibration)\n", tempADC, temp_C);
    
    // Note: This is a simplified calculation. For accurate external NTC temperature,
    // you would need to:
    // 1. Convert ADC to actual voltage
    // 2. Calculate NTC resistance using voltage divider
    // 3. Apply NTC Beta equation for temperature
    
    return temp_C;
}

float AS8510::getInternalTemperature() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return -999.0;
    }
    
    // AS8510 DATASHEET REQUIREMENT: Internal temperature sensor MUST use PGA gain 5
    // Current library uses gain 25 by default, which causes 5x amplification and saturation
    // Solution: Temporarily switch to gain 5, measure, then restore original gain
    
    uint8_t originalACH = readRegister(AS8510_ACH_CTL_REG);
    uint8_t originalPGA = readRegister(AS8510_PGA_CTL_REG);
    
    if (_verboseLogging) {
        Serial.println("AS8510: Internal temperature measurement using datasheet-required gain 5");
    }
    
    // Step 1: Configure PGA for gain 5 (datasheet requirement for internal temp sensor)
    // Temporarily store current voltage gain and switch to Gain5 for temperature measurement
    Gain originalVoltageGain = _voltageGain;
    _voltageGain = Gain::Gain5; // Set to datasheet-required gain 5
    
    // Calculate PGA register with gain 5 for voltage channel
    uint8_t tempPGA = calculateReg0x13();
    
    if (_verboseLogging) {
        Serial.printf("AS8510: Temporarily setting PGA to gain 5 for internal temp (PGA=0x%02X)\n", tempPGA);
    }
    writeRegister(AS8510_PGA_CTL_REG, tempPGA);
    delay(50); // Allow PGA to settle (datasheet: first sample after gain change is invalid)
    
    // Step 2: Configure MUX for internal temperature sensor
    // From datasheet: D[7:6] = 11 = Internal temperature channel
    // D[4] = 0 = No current source needed for internal temp sensor
    uint8_t internalTempMux = ACH_CTL_INTERNAL_TEMP_CHANNEL;
    
    if (_verboseLogging) {
        Serial.printf("AS8510: Configuring internal temperature sensor (ACH_CTL=0x%02X)\n", internalTempMux);
    }
    writeRegister(AS8510_ACH_CTL_REG, internalTempMux);
    delay(10);  // Allow MUX to settle
    
    // Step 3: Wait for voltage channel to be ready and discard first invalid sample
    int retries = 20;
    while (!isVoltageChannelUpdated() && retries > 0) {
        delay(5);
        retries--;
    }
    
    if (retries > 0) {
        // Discard first sample (invalid after gain change)
        readRegisters(ADDR_VOLTAGE, _data, 2);
        delay(10);
        
        // Wait for second (valid) sample
        retries = 20;
        while (!isVoltageChannelUpdated() && retries > 0) {
            delay(5);
            retries--;
        }
    }
    
    if (retries == 0) {
        Serial.println("AS8510: Internal temp measurement timeout");
        // Restore original settings
        _voltageGain = originalVoltageGain;
        writeRegister(AS8510_PGA_CTL_REG, originalPGA);
        writeRegister(AS8510_ACH_CTL_REG, originalACH);
        return -999.0;
    }
    
    // Step 4: Read internal temperature sensor data (now at correct gain 5)
    readRegisters(ADDR_VOLTAGE, _data, 2);
    uint16_t tempADC = (_data[0] << 8) | _data[1];
    
    // Step 5: Restore original PGA and MUX settings
    _voltageGain = originalVoltageGain; // Restore original voltage gain setting
    writeRegister(AS8510_PGA_CTL_REG, originalPGA);
    writeRegister(AS8510_ACH_CTL_REG, originalACH);
    delay(10); // Allow restoration to settle
    
    if (_verboseLogging) {
        Serial.printf("AS8510: Internal temp ADC at gain 5: 0x%04X (%u)\n", tempADC, tempADC);
    }
    
    // Step 6: Check for saturation (should not happen at gain 5)
    if (tempADC >= 65535) {
        if (_verboseLogging) {
            Serial.printf("AS8510: Internal temp sensor still saturated at gain 5: 0x%04X\n", tempADC);
        }
        return -999.0;
    }
    
    // Step 7: AS8510 Internal Temperature Conversion (from datasheet)
    // Temperature sensor slope: 27 Digits/°C at gain 5
    // Temperature sensor output at 65°C: 41807 Digits (typical) at gain 5
    // NO GAIN CORRECTION NEEDED - we measured at the correct gain 5
    
    float tempSlopeDigitsPerDegC = 27.0; // Digits/°C at gain 5
    float tempAt65C_Digits = 41807.0;    // Typical output at 65°C and gain 5
    float referenceTemp = 65.0;          // Reference temperature
    
    // Temperature calculation: T = T_ref + (ADC - ADC_ref) / slope
    float temp_C = referenceTemp + ((float)tempADC - tempAt65C_Digits) / tempSlopeDigitsPerDegC;
    
    if (_verboseLogging) {
        Serial.printf("AS8510: Internal temp: ADC=%u (gain 5), T=%.1f°C\n", tempADC, temp_C);
    }
    
    // Sanity check for reasonable IC operating temperature
    if (temp_C < -45.0 || temp_C > 130.0) {
        if (_verboseLogging) {
            Serial.printf("AS8510: WARNING - Internal temperature out of datasheet range: %.1f°C\n", temp_C);
            Serial.printf("AS8510: Datasheet range: -40°C to 125°C\n");
        }
    }
    
    return temp_C;
}

bool AS8510::isCurrentChannelUpdated() {
    uint8_t status = readRegister(AS8510_STATUS_REG);
    return (status & STATUS_CURRENT_CHANNEL_UPDATED) != 0;
}

bool AS8510::isVoltageChannelUpdated() {
    uint8_t status = readRegister(AS8510_STATUS_REG);
    return (status & STATUS_VOLTAGE_CHANNEL_UPDATED) != 0;
}

uint8_t AS8510::getStatus() {
    return readRegister(AS8510_STATUS_REG);
}

void AS8510::printStatus() {
          uint8_t status = getStatus();
    // Status checked
}

uint8_t AS8510::getErrorCodes() {
    return readRegister(AS8510_STATUS_REG);
}

uint8_t AS8510::getSaturationFlags() {
    return readRegister(AS8510_STATUS_REG_2);
}

void AS8510::printErrorCodes() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return;
    }
    
    uint8_t status = getErrorCodes();
    Serial.println("┌─── AS8510 Error Codes (Status Register 0x04) ───");
    Serial.printf("│ Raw Status: 0x%02X\n", status);
    Serial.println("├─── Fault Conditions ───");
    
    if (status & STATUS_OVERCURRENT_FAULT) {
        Serial.println("│ ⚠️  OVERCURRENT FAULT detected");
    }
    if (status & STATUS_OVERVOLTAGE_FAULT) {
        Serial.println("│ ⚠️  OVERVOLTAGE FAULT detected");
    }
    if (status & STATUS_UNDERVOLTAGE_FAULT) {
        Serial.println("│ ⚠️  UNDERVOLTAGE FAULT detected");
    }
    if (status & STATUS_OVERTEMPERATURE_FAULT) {
        Serial.println("│ ⚠️  OVERTEMPERATURE FAULT detected");
    }
    if (status & STATUS_COMMUNICATION_ERROR) {
        Serial.println("│ ⚠️  COMMUNICATION ERROR detected");
    }
    
    Serial.println("├─── Channel Status ───");
    if (status & STATUS_CURRENT_CHANNEL_UPDATED) {
        Serial.println("│ ✅ Current channel data ready");
    } else {
        Serial.println("│ ⏳ Current channel data not ready");
    }
    if (status & STATUS_VOLTAGE_CHANNEL_UPDATED) {
        Serial.println("│ ✅ Voltage channel data ready");
  } else {
        Serial.println("│ ⏳ Voltage channel data not ready");
    }
    if (status & STATUS_TEMPERATURE_READY) {
        Serial.println("│ ✅ Temperature data ready");
    } else {
        Serial.println("│ ⏳ Temperature data not ready");
    }
    
    // Overall system status
    uint8_t faultMask = STATUS_OVERCURRENT_FAULT | STATUS_OVERVOLTAGE_FAULT | 
                        STATUS_UNDERVOLTAGE_FAULT | STATUS_OVERTEMPERATURE_FAULT | 
                        STATUS_COMMUNICATION_ERROR;
    
    Serial.println("├─── Overall Status ───");
    if (status & faultMask) {
        Serial.println("│ 🚨 FAULT CONDITIONS DETECTED");
    } else {
        Serial.println("│ ✅ No fault conditions");
    }
    
    Serial.println("└─────────────────────────────────────────");
}

void AS8510::printSaturationFlags() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return;
    }
    
    uint8_t status2 = getSaturationFlags();
    Serial.println("┌─── AS8510 Saturation Flags (Status Register 0x44) ───");
    Serial.printf("│ Raw Status: 0x%02X\n", status2);
    Serial.println("├─── Saturation Conditions ───");
    
    if (status2 & STATUS2_CURRENT_SATURATION) {
        Serial.println("│ ⚠️  CURRENT SATURATION detected");
    }
    if (status2 & STATUS2_VOLTAGE_SATURATION) {
        Serial.println("│ ⚠️  VOLTAGE SATURATION detected");
    }
    if (status2 & STATUS2_TEMP_SATURATION) {
        Serial.println("│ ⚠️  TEMPERATURE SATURATION detected");
    }
    if (status2 & STATUS2_PGA_SATURATION) {
        Serial.println("│ ⚠️  PGA SATURATION detected");
    }
    
    Serial.println("├─── ADC Conditions ───");
    if (status2 & STATUS2_ADC_OVERFLOW) {
        Serial.println("│ ⚠️  ADC OVERFLOW detected");
    }
    if (status2 & STATUS2_ADC_UNDERFLOW) {
        Serial.println("│ ⚠️  ADC UNDERFLOW detected");
    }
    
    Serial.println("├─── System Conditions ───");
    if (status2 & STATUS2_CLOCK_ERROR) {
        Serial.println("│ ⚠️  CLOCK ERROR detected");
    }
    if (status2 & STATUS2_POWER_SUPPLY_ERROR) {
        Serial.println("│ ⚠️  POWER SUPPLY ERROR detected");
    }
    
    // Overall saturation status
    uint8_t saturationMask = STATUS2_CURRENT_SATURATION | STATUS2_VOLTAGE_SATURATION | 
                             STATUS2_TEMP_SATURATION | STATUS2_PGA_SATURATION |
                             STATUS2_ADC_OVERFLOW | STATUS2_ADC_UNDERFLOW |
                             STATUS2_CLOCK_ERROR | STATUS2_POWER_SUPPLY_ERROR;
    
    Serial.println("├─── Overall Status ───");
    if (status2 & saturationMask) {
        Serial.println("│ 🚨 SATURATION/ERROR CONDITIONS DETECTED");
    } else {
        Serial.println("│ ✅ No saturation/error conditions");
    }
    
    Serial.println("└─────────────────────────────────────────");
}

void AS8510::printAllDiagnostics() {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return;
    }
    
    Serial.println("╔═══════════════════════════════════════════════════════════════╗");
    Serial.println("║                    AS8510 COMPLETE DIAGNOSTICS                ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════╝");
    Serial.println();
    
    // Print error codes
    printErrorCodes();
    Serial.println();
    
    // Print saturation flags
    printSaturationFlags();
    Serial.println();
    
    // Print additional diagnostic information
    Serial.println("┌─── Additional Diagnostic Information ───");
    Serial.printf("│ Device initialized: %s\n", _isInitialized ? "YES" : "NO");
    Serial.printf("│ Current gain: %s\n", 
                  _currentGain == Gain::Gain5 ? "5" :
                  _currentGain == Gain::Gain25 ? "25" :
                  _currentGain == Gain::Gain40 ? "40" :
                  _currentGain == Gain::Gain100 ? "100" : "Unknown");
    Serial.printf("│ Voltage gain: %s\n", 
                  _voltageGain == Gain::Gain5 ? "5" :
                  _voltageGain == Gain::Gain25 ? "25" :
                  _voltageGain == Gain::Gain40 ? "40" :
                  _voltageGain == Gain::Gain100 ? "100" : "Unknown");
    Serial.printf("│ Shunt resistance: %.9f Ω\n", _shuntResistance);
    
    // Read key registers
    uint8_t modCtl = readRegister(AS8510_MOD_CTL_REG);
    uint8_t pgaCtl = readRegister(AS8510_PGA_CTL_REG);
    uint8_t achCtl = readRegister(AS8510_ACH_CTL_REG);
    
    Serial.println("├─── Key Register Values ───");
    Serial.printf("│ Mode Control (0x0A): 0x%02X %s\n", modCtl, 
                  (modCtl & MOD_CTL_START) ? "[STARTED]" : "[STOPPED]");
    Serial.printf("│ PGA Control (0x13):  0x%02X\n", pgaCtl);
    Serial.printf("│ Channel Control (0x17): 0x%02X\n", achCtl);
    
    Serial.println("└─────────────────────────────────────────");
    Serial.println();
}

void AS8510::startDevice() {
    Serial.println("AS8510: Explicit device start requested...");
    
    // Use the exact same approach as the Rust library
    writeRegister(AS8510_MOD_CTL_REG, MOD_CTL_START);
    delay(100);
    
    uint8_t modCtl = readRegister(AS8510_MOD_CTL_REG);
    Serial.printf("AS8510: Mode Control after start: 0x%02X\n", modCtl);
    
    if (modCtl & MOD_CTL_START) {
        Serial.println("AS8510: SUCCESS - START bit is set!");
        _isInitialized = true;
  } else {
        Serial.println("AS8510: FAILED - START bit not set");
        _isInitialized = false;
    }
}

int16_t AS8510::readRawADC(uint8_t channel) {
    if (!_isInitialized) {
        Serial.println("AS8510: ERROR - Device not initialized");
        return 0;
    }
    
    // Read current data (2 bytes) 
    readRegisters(ADDR_CURRENT, _data, 2);
    
    // Convert to signed 16-bit value
    int16_t rawADC = (int16_t)(_data[0] << 8 | _data[1]);
    
    Serial.printf("AS8510: Raw ADC: 0x%04X (%d)\n", rawADC, rawADC);
    
    return rawADC;
}



#endif // AS8510_H
