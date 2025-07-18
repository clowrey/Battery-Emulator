# ESPHome Battery Emulator Watchdog

This ESPHome project creates a monitoring system that reads Serial API data from the Battery Emulator and publishes all battery parameters to Home Assistant.

## Overview

The Battery Emulator Watchdog:
- Connects to the Battery Emulator's Serial API output (UART)
- Parses JSON data in the format: `SERIAL_API:<data_type>:<json_message>`
- Creates Home Assistant sensors for all battery parameters
- Provides real-time monitoring of battery status, cell voltages, events, and more
- Supports dual battery configurations
- Monitors connectivity and provides status indicators

## Hardware Requirements

- ESP32 development board (ESP32-DevKit or similar)
- USB cable for power and programming
- Jumper wires for UART connection
- Battery Emulator with Serial API enabled

## Wiring

Connect the ESP32 to the Battery Emulator's Serial API output:

```
Battery Emulator → ESP32
Pin 14 (TX)     → GPIO3 (RX)
GND             → GND
```

**Note:** The Battery Emulator uses pin 14 for Serial API TX by default (configurable in `USER_SETTINGS.h`). The ESP32 doesn't need to send data back, so only RX connection is required.

## Software Setup

### 1. Configure Battery Emulator

Ensure the following settings are enabled in `Software/USER_SETTINGS.h`:

```cpp
#define SERIAL_API                    // Enable Serial API
#define SERIAL_API_DEDICATED_SERIAL   // Use dedicated serial port
#define SERIAL_API_TX_PIN 14          // TX pin (connect to ESP32 GPIO3)
#define SERIAL_API_RX_PIN 15          // RX pin (not used but required)
#define SERIAL_API_BAUDRATE 115200    // Baud rate
```

### 2. Setup ESPHome

1. Install ESPHome on your system
2. Copy `secrets.yaml.template` to `secrets.yaml`
3. Edit `secrets.yaml` with your WiFi credentials and other settings:

```yaml
wifi_ssid: "Your_WiFi_SSID"
wifi_password: "Your_WiFi_Password"
ap_password: "fallback-hotspot-password"
api_encryption_key: "your-32-character-encryption-key"
ota_password: "your-ota-password"
```

### 3. Compile and Upload

```bash
cd esphome-watchdog
esphome run battery-emulator-watchdog.yaml
```

## Available Sensors

### Main Battery Data
- **SOC (State of Charge)** - Battery percentage (%)
- **SOC Real** - Real battery percentage without scaling (%)
- **State of Health** - Battery health percentage (%)
- **Temperature Min/Max** - Battery temperature range (°C)
- **CPU Temperature** - Battery Emulator CPU temperature (°C)
- **Battery Power** - Current power flow (W, positive = charging, negative = discharging)
- **Battery Current** - Current flow (A)
- **Battery Voltage** - Pack voltage (V)
- **Cell Max/Min Voltage** - Individual cell voltage extremes (V)
- **Cell Voltage Delta** - Difference between max and min cell voltages (mV)
- **Total Capacity** - Total battery capacity (Wh)
- **Remaining Capacity** - Current available energy (Wh)
- **Max Charge/Discharge Power** - Power limits (W)
- **Total Charged/Discharged Energy** - Lifetime energy counters (Wh)
- **Balancing Active Cells** - Number of cells currently balancing

### Individual Cell Voltages
- **Cell 1-96 Voltage** - Individual cell voltages for Tesla Model 3 (configurable)

### Second Battery (if configured)
- **Battery 2 SOC** - Second battery state of charge (%)
- **Battery 2 Power** - Second battery power (W)
- **Battery 2 Voltage** - Second battery voltage (V)

### Status Information
- **BMS Status** - Battery Management System status
- **Emulator Pause Status** - Current pause/run state
- **Last Event** - Most recent battery event/error message
- **Event Severity** - Severity level of last event

### Connectivity
- **Battery Connected** - Battery communication status
- **Emulator Online** - Watchdog connectivity to emulator

## Data Types Monitored

The component monitors these Serial API data types:

1. **info** - Main battery parameters (SOC, voltage, current, temperatures, etc.)
2. **spec_data** - Individual cell voltages array
3. **balancing_data** - Cell balancing status array
4. **events** - Error messages and events
5. **status** - Online/offline status

## Configuration Options

### UART Configuration
```yaml
uart:
  id: battery_uart
  tx_pin: GPIO1  # Not used
  rx_pin: GPIO3  # Connect to Battery Emulator TX
  baud_rate: 115200
```

### Update Intervals
- Main data: Every 5 seconds (matches Battery Emulator)
- Check timer: 800ms (matches Battery Emulator)
- Connectivity timeout: 15 seconds

### Customizing Sensors

You can enable/disable individual sensors by commenting them out in the YAML configuration. For example, to disable second battery sensors:

```yaml
# Second battery sensors (comment out if not needed)
# - platform: battery_monitor
#   soc_2:
#     name: "Battery 2 SOC"
```

### Adding More Cell Voltages

To monitor more individual cell voltages, add them to the `cell_voltages` section:

```yaml
sensor:
  - platform: battery_monitor
    cell_voltages:
      - name: "Cell 1 Voltage"
        index: 0
      - name: "Cell 2 Voltage"
        index: 1
      # Add more cells as needed (up to 96 for Tesla Model 3)
```

## Home Assistant Integration

After flashing the ESP32:

1. The device will appear in Home Assistant automatically via the API
2. All sensors will be available under the device name
3. Use the sensors in dashboards, automations, and scripts
4. Set up alerts for critical values (temperature, voltage deviations, etc.)

### Example Dashboard Card

```yaml
type: entities
title: Battery Monitor
entities:
  - sensor.battery_soc
  - sensor.battery_voltage
  - sensor.battery_current
  - sensor.battery_power
  - sensor.battery_temperature_min
  - sensor.battery_temperature_max
  - sensor.cell_max_voltage
  - sensor.cell_min_voltage
  - sensor.cell_voltage_delta
  - binary_sensor.battery_connected
  - binary_sensor.emulator_online
```

## Troubleshooting

### No Data Received
1. Check UART wiring (TX → RX, GND → GND)
2. Verify Serial API is enabled in Battery Emulator
3. Check baud rate matches (115200)
4. Monitor ESPHome logs for received data

### Connection Issues
1. Verify WiFi credentials in `secrets.yaml`
2. Check Home Assistant API encryption key
3. Ensure ESP32 is on same network as Home Assistant

### Data Parsing Errors
1. Check ESPHome logs for JSON parsing errors
2. Verify Serial API format from Battery Emulator
3. Ensure buffer isn't overflowing (check line length)

## Advanced Configuration

### Multiple Battery Emulators
To monitor multiple Battery Emulators, create separate ESP32 devices with different device names and pin configurations.

### Custom Alerts
Set up Home Assistant automations for critical conditions:

```yaml
automation:
  - alias: "Battery Temperature Alert"
    trigger:
      platform: numeric_state
      entity_id: sensor.battery_temperature_max
      above: 45
    action:
      service: notify.mobile_app
      data:
        message: "Battery temperature high: {{ states('sensor.battery_temperature_max') }}°C"
```

### Data Logging
Use Home Assistant's built-in recorder or InfluxDB for long-term data storage and analysis.

## Development

### File Structure
```
esphome-watchdog/
├── battery-emulator-watchdog.yaml    # Main ESPHome configuration
├── secrets.yaml.template             # Secrets template
├── components/
│   └── battery_monitor/
│       ├── __init__.py               # Component registration
│       ├── battery_monitor.h         # C++ header
│       ├── battery_monitor.cpp       # C++ implementation
│       ├── sensor.py                 # Sensor platform
│       ├── text_sensor.py           # Text sensor platform
│       └── binary_sensor.py         # Binary sensor platform
└── README.md                        # This file
```

### Contributing
1. Test thoroughly with different battery types
2. Follow ESPHome coding standards
3. Update documentation for new features
4. Consider backward compatibility

## License

This project follows the same license as the main Battery Emulator project. 