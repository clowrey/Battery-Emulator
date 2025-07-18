# 1 "C:\\Users\\hilo9\\AppData\\Local\\Temp\\tmpkw0keu61"
#include <Arduino.h>
# 1 "C:/Users/hilo9/Documents/GitHub/Battery-Emulator/Software/Software.ino"


#include "HardwareSerial.h"
#include "USER_SECRETS.h"
#include "USER_SETTINGS.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "src/communication/Transmitter.h"
#include "src/communication/can/comm_can.h"
#include "src/communication/contactorcontrol/comm_contactorcontrol.h"
#include "src/communication/equipmentstopbutton/comm_equipmentstopbutton.h"
#include "src/communication/nvm/comm_nvm.h"
#include "src/communication/precharge_control/precharge_control.h"
#include "src/communication/rs485/comm_rs485.h"
#include "src/datalayer/datalayer.h"
#include "src/devboard/sdcard/sdcard.h"
#include "src/devboard/utils/events.h"
#include "src/devboard/utils/led_handler.h"
#include "src/devboard/utils/logging.h"
#include "src/devboard/utils/timer.h"
#include "src/devboard/utils/value_mapping.h"
#include "src/include.h"
#ifndef AP_PASSWORD
#error \
    "Initial setup not completed, USER_SECRETS.h is missing. Please rename the file USER_SECRETS.TEMPLATE.h to USER_SECRETS.h and fill in the required credentials. This file is ignored by version control to keep sensitive information private."
#endif
#ifdef WIFI
#include "src/devboard/wifi/wifi.h"
#ifdef WEBSERVER
#include "src/devboard/webserver/webserver.h"
#ifdef MDNSRESPONDER
#include <ESPmDNS.h>
#endif
#else
#ifdef MDNSRESPONDER
#error WEBSERVER needs to be enabled for MDNSRESPONDER!
#endif
#endif
#ifdef MQTT
#include "src/devboard/mqtt/mqtt.h"
#endif
#ifdef SERIAL_API
#include "src/communication/serial_api/serial_api.h"
#endif
#endif
#ifdef PERIODIC_BMS_RESET_AT
#include "src/devboard/utils/ntp_time.h"
#endif
volatile unsigned long long bmsResetTimeOffset = 0;


const char* version_number = "8.15.0";


volatile unsigned long currentMillis = 0;
unsigned long previousMillis10ms = 0;
unsigned long previousMillisUpdateVal = 0;
unsigned long lastMillisOverflowCheck = 0;
#ifdef FUNCTION_TIME_MEASUREMENT

MyTimer core_task_timer_10s(INTERVAL_10_S);
#endif
TaskHandle_t main_loop_task;
TaskHandle_t connectivity_loop_task;
TaskHandle_t logging_loop_task;
TaskHandle_t mqtt_loop_task;
#ifdef SERIAL_API
TaskHandle_t serial_api_loop_task;
#endif

Logging logging;
void setup();
void loop();
void logging_loop(void*);
void connectivity_loop(void*);
void mqtt_loop(void*);
void serial_api_loop(void*);
void register_transmitter(Transmitter* transmitter);
void core_loop(void*);
void init_serial();
void check_interconnect_available();
void update_calculated_values();
void check_reset_reason();
#line 78 "C:/Users/hilo9/Documents/GitHub/Battery-Emulator/Software/Software.ino"
void setup() {
  init_serial();


  logging.printf("Battery emulator %s build " __DATE__ " " __TIME__ "\n", version_number);

  init_events();

  init_stored_settings();

#ifdef WIFI
  xTaskCreatePinnedToCore((TaskFunction_t)&connectivity_loop, "connectivity_loop", 4096, NULL, TASK_CONNECTIVITY_PRIO,
                          &connectivity_loop_task, WIFI_CORE);
#endif

#if defined(LOG_CAN_TO_SD) || defined(LOG_TO_SD)
  xTaskCreatePinnedToCore((TaskFunction_t)&logging_loop, "logging_loop", 4096, NULL, TASK_CONNECTIVITY_PRIO,
                          &logging_loop_task, WIFI_CORE);
#endif

#ifdef PRECHARGE_CONTROL
  init_precharge_control();
#endif

  setup_charger();
  setup_inverter();
  setup_battery();
  setup_can_shunt();


  init_CAN();

  init_contactors();

  init_rs485();

#ifdef EQUIPMENT_STOP_BUTTON
  init_equipment_stop_button();
#endif


  pinMode(0, INPUT_PULLUP);

  check_reset_reason();


  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = INTERVAL_5_S,
      .idle_core_mask = (1 << CORE_FUNCTION_CORE) | (1 << WIFI_CORE),
      .trigger_panic = true
  };



#ifdef MQTT
  init_mqtt();

  xTaskCreatePinnedToCore((TaskFunction_t)&mqtt_loop, "mqtt_loop", 4096, NULL, TASK_MQTT_PRIO, &mqtt_loop_task,
                          WIFI_CORE);
#endif

#ifdef SERIAL_API

  init_serial_api();
  xTaskCreatePinnedToCore((TaskFunction_t)&serial_api_loop, "serial_api_loop", 4096, NULL, TASK_MQTT_PRIO, &serial_api_loop_task,
                          WIFI_CORE);
#endif

  xTaskCreatePinnedToCore((TaskFunction_t)&core_loop, "core_loop", 4096, NULL, TASK_CORE_PRIO, &main_loop_task,
                          CORE_FUNCTION_CORE);
#ifdef PERIODIC_BMS_RESET_AT
  bmsResetTimeOffset = getTimeOffsetfromNowUntil(PERIODIC_BMS_RESET_AT);
  if (bmsResetTimeOffset == 0) {
    set_event(EVENT_PERIODIC_BMS_RESET_AT_INIT_FAILED, 0);
  } else {
    set_event(EVENT_PERIODIC_BMS_RESET_AT_INIT_SUCCESS, 0);
  }
#endif

  DEBUG_PRINTF("setup() complete\n");
}


void loop() {}

#if defined(LOG_CAN_TO_SD) || defined(LOG_TO_SD)
void logging_loop(void*) {

  init_logging_buffers();
  init_sdcard();

  while (true) {
#ifdef LOG_TO_SD
    write_log_to_sdcard();
#endif
#ifdef LOG_CAN_TO_SD
    write_can_frame_to_sdcard();
#endif
  }
}
#endif

#ifdef WIFI
void connectivity_loop(void*) {
  esp_task_wdt_add(NULL);

  init_WiFi();

#ifdef WEBSERVER

  init_webserver();
#endif
#ifdef MDNSRESPONDER
  init_mDNS();
#endif

  while (true) {
    START_TIME_MEASUREMENT(wifi);
    wifi_monitor();
#ifdef WEBSERVER
    ota_monitor();
#endif
    END_TIME_MEASUREMENT_MAX(wifi, datalayer.system.status.wifi_task_10s_max_us);

    esp_task_wdt_reset();
    delay(1);
  }
}
#endif

#ifdef MQTT
void mqtt_loop(void*) {
  esp_task_wdt_add(NULL);

  while (true) {
    START_TIME_MEASUREMENT(mqtt);
    mqtt_loop();
    END_TIME_MEASUREMENT_MAX(mqtt, datalayer.system.status.mqtt_task_10s_max_us);
    esp_task_wdt_reset();
    delay(1);
  }
}
#endif

#ifdef SERIAL_API
void serial_api_loop(void*) {
  esp_task_wdt_add(NULL);

  while (true) {
    START_TIME_MEASUREMENT(serial_api);
    serial_api_loop();
    END_TIME_MEASUREMENT_MAX(serial_api, datalayer.system.status.mqtt_task_10s_max_us);
    esp_task_wdt_reset();
    delay(1);
  }
}
#endif

static std::list<Transmitter*> transmitters;

void register_transmitter(Transmitter* transmitter) {
  transmitters.push_back(transmitter);
  DEBUG_PRINTF("transmitter registered, total: %d\n", transmitters.size());
}

void core_loop(void*) {
  esp_task_wdt_add(NULL);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1);
  led_init();

  while (true) {

    START_TIME_MEASUREMENT(all);
    START_TIME_MEASUREMENT(comm);
#ifdef EQUIPMENT_STOP_BUTTON
    monitor_equipment_stop_button();
#endif


    receive_can();
    receive_rs485();

    END_TIME_MEASUREMENT_MAX(comm, datalayer.system.status.time_comm_us);
#ifdef WEBSERVER
    START_TIME_MEASUREMENT(ota);
    ElegantOTA.loop();
    END_TIME_MEASUREMENT_MAX(ota, datalayer.system.status.time_ota_us);
#endif


    currentMillis = millis();
    if (currentMillis - previousMillis10ms >= INTERVAL_10_MS) {
      if ((currentMillis - previousMillis10ms >= INTERVAL_10_MS_DELAYED) && (currentMillis > BOOTUP_TIME)) {
        set_event(EVENT_TASK_OVERRUN, (currentMillis - previousMillis10ms));
      }
      previousMillis10ms = currentMillis;
#ifdef FUNCTION_TIME_MEASUREMENT
      START_TIME_MEASUREMENT(time_10ms);
#endif
      led_exe();
      handle_contactors();
#ifdef PRECHARGE_CONTROL
      handle_precharge_control(currentMillis);
#endif
#ifdef FUNCTION_TIME_MEASUREMENT
      END_TIME_MEASUREMENT_MAX(time_10ms, datalayer.system.status.time_10ms_us);
#endif
    }

    if (currentMillis - previousMillisUpdateVal >= INTERVAL_1_S) {
      previousMillisUpdateVal = currentMillis;
#ifdef FUNCTION_TIME_MEASUREMENT
      START_TIME_MEASUREMENT(time_values);
#endif
      update_pause_state();


      if (battery) {
        battery->update_values();
      }

      if (battery2) {
        battery2->update_values();
        check_interconnect_available();
      }
      update_calculated_values();
      update_machineryprotection();


      if (inverter) {
        inverter->update_values();
      }

#ifdef FUNCTION_TIME_MEASUREMENT
      END_TIME_MEASUREMENT_MAX(time_values, datalayer.system.status.time_values_us);
#endif
    }
#ifdef FUNCTION_TIME_MEASUREMENT
    START_TIME_MEASUREMENT(cantx);
#endif


    for (auto& transmitter : transmitters) {
      transmitter->transmit(currentMillis);
    }

#ifdef FUNCTION_TIME_MEASUREMENT
    END_TIME_MEASUREMENT_MAX(cantx, datalayer.system.status.time_cantx_us);
    END_TIME_MEASUREMENT_MAX(all, datalayer.system.status.core_task_10s_max_us);
#endif
#ifdef FUNCTION_TIME_MEASUREMENT
    if (datalayer.system.status.core_task_10s_max_us > datalayer.system.status.core_task_max_us) {

      datalayer.system.status.core_task_max_us = datalayer.system.status.core_task_10s_max_us;

      datalayer.system.status.time_snap_comm_us = datalayer.system.status.time_comm_us;
      datalayer.system.status.time_snap_10ms_us = datalayer.system.status.time_10ms_us;
      datalayer.system.status.time_snap_values_us = datalayer.system.status.time_values_us;
      datalayer.system.status.time_snap_cantx_us = datalayer.system.status.time_cantx_us;
      datalayer.system.status.time_snap_ota_us = datalayer.system.status.time_ota_us;
    }

    datalayer.system.status.core_task_max_us =
        MAX(datalayer.system.status.core_task_10s_max_us, datalayer.system.status.core_task_max_us);
    if (core_task_timer_10s.elapsed()) {
      datalayer.system.status.time_ota_us = 0;
      datalayer.system.status.time_comm_us = 0;
      datalayer.system.status.time_10ms_us = 0;
      datalayer.system.status.time_values_us = 0;
      datalayer.system.status.time_cantx_us = 0;
      datalayer.system.status.core_task_10s_max_us = 0;
      datalayer.system.status.wifi_task_10s_max_us = 0;
      datalayer.system.status.mqtt_task_10s_max_us = 0;
    }
#endif
    esp_task_wdt_reset();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void init_serial() {

  Serial.begin(115200);
  while (!Serial) {}
#ifdef DEBUG_VIA_USB
  Serial.println("__ OK __");
#endif
}

void check_interconnect_available() {
  if (datalayer.battery.status.voltage_dV == 0 || datalayer.battery2.status.voltage_dV == 0) {
    return;
  }

  uint16_t voltage_diff = abs(datalayer.battery.status.voltage_dV - datalayer.battery2.status.voltage_dV);

  if (voltage_diff <= 30) {
    clear_event(EVENT_VOLTAGE_DIFFERENCE);
    if (datalayer.battery.status.bms_status == FAULT) {

      datalayer.system.status.battery2_allowed_contactor_closing = false;
    } else {
      datalayer.system.status.battery2_allowed_contactor_closing = true;
    }
  } else {
    set_event(EVENT_VOLTAGE_DIFFERENCE, (uint8_t)(voltage_diff / 10));
  }
}

void update_calculated_values() {

  datalayer.system.info.CPU_temperature = temperatureRead();


  if (datalayer.battery.status.voltage_dV > 10) {

    datalayer.battery.status.max_charge_current_dA =
        ((datalayer.battery.status.max_charge_power_W * 100) / datalayer.battery.status.voltage_dV);
    datalayer.battery.status.max_discharge_current_dA =
        ((datalayer.battery.status.max_discharge_power_W * 100) / datalayer.battery.status.voltage_dV);
  }

  if (datalayer.battery.status.max_charge_current_dA > datalayer.battery.settings.max_user_set_charge_dA) {
    datalayer.battery.status.max_charge_current_dA = datalayer.battery.settings.max_user_set_charge_dA;
    datalayer.battery.settings.user_settings_limit_charge = true;
  } else {
    datalayer.battery.settings.user_settings_limit_charge = false;
  }
  if (datalayer.battery.status.max_discharge_current_dA > datalayer.battery.settings.max_user_set_discharge_dA) {
    datalayer.battery.status.max_discharge_current_dA = datalayer.battery.settings.max_user_set_discharge_dA;
    datalayer.battery.settings.user_settings_limit_discharge = true;
  } else {
    datalayer.battery.settings.user_settings_limit_discharge = false;
  }

  datalayer.battery.status.active_power_W =
      (datalayer.battery.status.current_dA * (datalayer.battery.status.voltage_dV / 100));


  if (datalayer.battery.status.current_dA == 0) {
    if (datalayer.battery.status.max_discharge_current_dA > 0) {

      datalayer.battery.settings.inverter_limits_discharge = true;
    } else {
      datalayer.battery.settings.inverter_limits_discharge = false;
    }
    if (datalayer.battery.status.max_charge_current_dA > 0) {

      datalayer.battery.settings.inverter_limits_charge = true;
    } else {
      datalayer.battery.settings.inverter_limits_charge = false;
    }
  } else if (datalayer.battery.status.current_dA < 0) {
    if (-datalayer.battery.status.current_dA < datalayer.battery.status.max_discharge_current_dA) {
      datalayer.battery.settings.inverter_limits_discharge = true;
    } else {
      datalayer.battery.settings.inverter_limits_discharge = false;
    }
  } else {

    if (datalayer.battery.status.current_dA < datalayer.battery.status.max_charge_current_dA) {
      datalayer.battery.settings.inverter_limits_charge = true;
    } else {
      datalayer.battery.settings.inverter_limits_charge = false;
    }
  }

  if (battery2) {

    datalayer.battery2.status.active_power_W =
        (datalayer.battery2.status.current_dA * (datalayer.battery2.status.voltage_dV / 100));
  }

  if (datalayer.battery.settings.soc_scaling_active) {
# 467 "C:/Users/hilo9/Documents/GitHub/Battery-Emulator/Software/Software.ino"
    int32_t delta_pct = datalayer.battery.settings.max_percentage - datalayer.battery.settings.min_percentage;
    int32_t clamped_soc = CONSTRAIN(datalayer.battery.status.real_soc, datalayer.battery.settings.min_percentage,
                                    datalayer.battery.settings.max_percentage);
    int32_t scaled_soc = 0;
    int32_t scaled_total_capacity = 0;
    if (delta_pct != 0) {
      scaled_soc = 10000 * (clamped_soc - datalayer.battery.settings.min_percentage) / delta_pct;
    }

    datalayer.battery.status.reported_soc = scaled_soc;


    if (datalayer.battery.info.total_capacity_Wh > 0 && datalayer.battery.status.real_soc > 0) {

      scaled_total_capacity = (datalayer.battery.info.total_capacity_Wh * delta_pct) / 10000;
      datalayer.battery.info.reported_total_capacity_Wh = scaled_total_capacity;


      datalayer.battery.status.reported_remaining_capacity_Wh = (scaled_total_capacity * scaled_soc) / 10000;

    } else {

      datalayer.battery.info.reported_total_capacity_Wh = datalayer.battery.info.total_capacity_Wh;
      datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery.status.remaining_capacity_Wh;
    }

    if (battery2) {

      if (datalayer.battery2.info.total_capacity_Wh > 0 && datalayer.battery.status.real_soc > 0) {

        datalayer.battery2.info.reported_total_capacity_Wh = scaled_total_capacity;

        datalayer.battery2.status.reported_remaining_capacity_Wh = (scaled_total_capacity * scaled_soc) / 10000;

      } else {

        datalayer.battery2.info.reported_total_capacity_Wh = datalayer.battery2.info.total_capacity_Wh;
        datalayer.battery2.status.reported_remaining_capacity_Wh = datalayer.battery2.status.remaining_capacity_Wh;
      }



      datalayer.battery.info.reported_total_capacity_Wh += datalayer.battery2.info.reported_total_capacity_Wh;
      datalayer.battery.status.reported_remaining_capacity_Wh +=
          datalayer.battery2.status.reported_remaining_capacity_Wh;
    }

  } else {
    datalayer.battery.status.reported_soc = datalayer.battery.status.real_soc;
    datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery.status.remaining_capacity_Wh;
    datalayer.battery.info.reported_total_capacity_Wh = datalayer.battery.info.total_capacity_Wh;

    if (battery2) {
      datalayer.battery2.status.reported_soc = datalayer.battery2.status.real_soc;
      datalayer.battery2.status.reported_remaining_capacity_Wh = datalayer.battery2.status.remaining_capacity_Wh;
      datalayer.battery2.info.reported_total_capacity_Wh = datalayer.battery2.info.total_capacity_Wh;
    }
  }

  if (battery2) {

    if (datalayer.battery.status.real_soc < 100) {
      datalayer.battery.status.reported_soc = datalayer.battery.status.real_soc;
      datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery.status.remaining_capacity_Wh;
    }
    if (datalayer.battery2.status.real_soc <
        100) {
      datalayer.battery.status.reported_soc = datalayer.battery2.status.real_soc;
      datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery2.status.remaining_capacity_Wh;
    }

    if (datalayer.battery.status.real_soc >
        9900) {
      datalayer.battery.status.reported_soc = datalayer.battery.status.real_soc;
      datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery.status.remaining_capacity_Wh;
    }
    if (datalayer.battery2.status.real_soc >
        9900) {
      datalayer.battery.status.reported_soc = datalayer.battery2.status.real_soc;
      datalayer.battery.status.reported_remaining_capacity_Wh = datalayer.battery2.status.remaining_capacity_Wh;
    }
  }

  if (currentMillis < lastMillisOverflowCheck) {
    datalayer.system.status.millisrolloverCount++;
  }
  lastMillisOverflowCheck = currentMillis;
}

void check_reset_reason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:
      set_event(EVENT_RESET_UNKNOWN, reason);
      break;
    case ESP_RST_POWERON:
      set_event(EVENT_RESET_POWERON, reason);
      break;
    case ESP_RST_EXT:
      set_event(EVENT_RESET_EXT, reason);
      break;
    case ESP_RST_SW:
      set_event(EVENT_RESET_SW, reason);
      break;
    case ESP_RST_PANIC:
      set_event(EVENT_RESET_PANIC, reason);
      break;
    case ESP_RST_INT_WDT:
      set_event(EVENT_RESET_INT_WDT, reason);
      break;
    case ESP_RST_TASK_WDT:
      set_event(EVENT_RESET_TASK_WDT, reason);
      break;
    case ESP_RST_WDT:
      set_event(EVENT_RESET_WDT, reason);
      break;
    case ESP_RST_DEEPSLEEP:
      set_event(EVENT_RESET_DEEPSLEEP, reason);
      break;
    case ESP_RST_BROWNOUT:
      set_event(EVENT_RESET_BROWNOUT, reason);
      break;
    case ESP_RST_SDIO:
      set_event(EVENT_RESET_SDIO, reason);
      break;
    case ESP_RST_USB:
      set_event(EVENT_RESET_USB, reason);
      break;
    case ESP_RST_JTAG:
      set_event(EVENT_RESET_JTAG, reason);
      break;
    case ESP_RST_EFUSE:
      set_event(EVENT_RESET_EFUSE, reason);
      break;
    case ESP_RST_PWR_GLITCH:
      set_event(EVENT_RESET_PWR_GLITCH, reason);
      break;
    case ESP_RST_CPU_LOCKUP:
      set_event(EVENT_RESET_CPU_LOCKUP, reason);
      break;
    default:
      break;
  }
}