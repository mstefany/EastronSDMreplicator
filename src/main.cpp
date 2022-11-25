#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <math.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <Syslog.h>
#include <WiFi.h>

#include <secret.h>

// https://support.innon.com/PowerMeters/SDM630-MOD-MID/Manual/SDM630-Modbus_Protocol.pdf
#define EASTRON_SERVER_ID 2
#define EASTRON_RX_TX_PIN 5
#define EASTRON_RX_PIN 18
#define EASTRON_TX_PIN 19
#define INFINISOLAR_SERVER_ID 2
#define INFINISOLAR_RX_TX_PIN 21
#define INFINISOLAR_RX_PIN 22
#define INFINISOLAR_TX_PIN 23

#define EASTRON_REG_COUNT 382
#define LOCAL_REG_COUNT 10
#define LOCAL_REG_START 10000
#define LOCAL_REG_GRIDOVERFLOW 10000
#define LOCAL_REG_GRIDIMPORT 10004
#define LOCAL_REG_GRIDEXPORT 10005
#define LOCAL_REG_SSRPOWER 10001
#define LOCAL_REG_DUTYCYCLE 10002
#define LOCAL_REG_MANUALCONTROL 10003

#define PWM1_GPIO 33
#define PWM1_CH 0
#define PWM1_RES 10
#define PWM1_FREQ 10

#define WIFI_TIMEOUT_MS 10000

#define APP_NAME "sdm-replicator"

#define MIN_PWM_POWER 200 // do not send less than 200 W
#define MAX_PWM_POWER 1775 // this should be measured before
#define MIN_PWM_DIFF 100 // do not act on small changes

IPAddress esp_ip(192, 168, 2, 15);
IPAddress esp_mask(255, 255, 255, 0);
IPAddress esp_gw(192, 168, 2, 1);
IPAddress esp_dns1(192, 168, 2, 1);

HardwareSerial EastronSerial(1);
HardwareSerial InfiniSolarSerial(2);

ModbusRTU EastronModbus;
ModbusRTU InfinisolarModbus;
ModbusIP EspModbus;

WiFiUDP udpClient;

Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);

TaskHandle_t infiniTaskHandle = NULL;
TaskHandle_t espTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
TaskHandle_t pwmTaskHandle = NULL;

int16_t ssrPower = 0;
int16_t ssrPowerLast = 0;
uint16_t dutyCycle = 0;

union 
{
  uint32_t x;
  float f;
} power1;

union 
{
  uint32_t x;
  float f;
} power2;

union 
{
  uint32_t x;
  float f;
} power3;

bool cbSomebodyConnected(IPAddress ip) {
  Serial.print("New TCP client connected: ");
  Serial.println(ip);

  int n = ip.toString().length();
  char char_array[n + 1];
  strcpy(char_array, ip.toString().c_str());
  if (WiFi.status() == WL_CONNECTED) {
    syslog.logf(LOG_INFO, "New TCP client connected: %s", char_array);
  }
  return true;
}

int calculateDutyCycle(int16_t desiredPower) {
  if (desiredPower >= MAX_PWM_POWER) {
    return pow(2, PWM1_RES);
  }

  double percentage = (double)desiredPower / (double)MAX_PWM_POWER;
  double fullArea = 2;
  double desiredArea = fullArea * percentage;

  int pwmResolution = pow(2, PWM1_RES);

  double width = PI / pwmResolution; // rectangular width for integral approximation

  double accumulated = 0;
  for (int i = 1; i < pwmResolution; i++) {
    accumulated += width * sin(i * width);

    if (accumulated >= desiredArea) {
      return i - 1;
    }
  }

  return 0; // should not ever reach
}

uint16_t cbRead(TRegister* reg, uint16_t val) {
  Serial.print("Read RAW#: ");
  Serial.println(reg->address.address);
  return val;
}

uint16_t cbWriteCoil(TRegister* reg, uint16_t val) {
  Serial.print("Write Coil RAW#: ");
  Serial.println(reg->address.address);
  Serial.print("Value:");
  Serial.println(COIL_BOOL(val));

  return val;
}

uint16_t cbWriteHolding(TRegister* reg, uint16_t val) {
  Serial.print("Write Holding RAW#: ");
  Serial.println(reg->address.address);
  Serial.print("Value:");
  Serial.println(val);

  return val;
}

void keepWifiTask(void *pvParameters) {
  Serial.println("Started keepWifiTask");
  for(;;) {
    if (WiFi.status() == WL_CONNECTED) {
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }

    WiFi.config(esp_ip, esp_gw, esp_mask, esp_dns1);

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {}

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi connection failed");
      vTaskDelay(20000 / portTICK_PERIOD_MS);
      continue;
    }

    Serial.println("WiFi connected");  
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    int n = WiFi.localIP().toString().length();
    char char_array[n + 1];
    strcpy(char_array, WiFi.localIP().toString().c_str());
    syslog.logf(LOG_INFO, "WiFi connected. IP address: %s", char_array);

    if (espTaskHandle) {
      xTaskNotifyGive(espTaskHandle);
    }

    if (otaTaskHandle) {
      xTaskNotifyGive(otaTaskHandle);
    }

    if (pwmTaskHandle) {
      xTaskNotifyGive(pwmTaskHandle);
    }
  } 
}

void eastronTask(void *pvParameters) {
  Serial.println("Started eastronTask");

  // Modbus RTU to power meter
  EastronSerial.begin(19200, SERIAL_8N1, EASTRON_RX_PIN, EASTRON_TX_PIN);
  EastronModbus.begin(&EastronSerial, EASTRON_RX_TX_PIN);
  EastronModbus.client();

  if (infiniTaskHandle) {
    xTaskNotifyGive(infiniTaskHandle);
  }

  if (pwmTaskHandle) {
    xTaskNotifyGive(pwmTaskHandle);
  }

  for(;;) {
    if (!EastronModbus.server()) {
      EastronModbus.pullIreg(EASTRON_SERVER_ID, 0, 0, 64);
      while(EastronModbus.server()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SERVER_ID, 64, 64, 64);
      while(EastronModbus.server()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SERVER_ID, 200, 200, 8);
      while(EastronModbus.server()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      /*
      EastronModbus.pullIreg(EASTRON_SERVER_ID, 224, 224, 46);
      while(EastronModbus.server()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SERVER_ID, 334, 334, 48);
      while(EastronModbus.server()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      */
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void infiniTask(void *pvParameters) {
  Serial.println("Started infiniTask");

  xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
                  ULONG_MAX, /* Reset the notification value to 0 on exit. */
                  NULL,
                  portMAX_DELAY );  /* Block indefinitely. */

  Serial.println("Unblocked infiniTask");

  // Modbus RTU to inverter
  InfiniSolarSerial.begin(19200, SERIAL_8N1, INFINISOLAR_RX_PIN, INFINISOLAR_TX_PIN);
  InfinisolarModbus.begin(&InfiniSolarSerial, INFINISOLAR_RX_TX_PIN);
  InfinisolarModbus.server(INFINISOLAR_SERVER_ID);
  InfinisolarModbus.addIreg(0, 0, EASTRON_REG_COUNT);

  for(;;) {
    InfinisolarModbus.task();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void mbServerTask(void *pvParameters) {
  Serial.println("Started mbServerTask");

  xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
                  ULONG_MAX, /* Reset the notification value to 0 on exit. */
                  NULL,
                  portMAX_DELAY );  /* Block indefinitely. */

  Serial.println("Unblocked mbServerTask");

  // Modbus IP for HA
  EspModbus.onConnect(cbSomebodyConnected);
  EspModbus.server();
  EspModbus.addIreg(0, 0, EASTRON_REG_COUNT);
  EspModbus.addIreg(LOCAL_REG_START, 0, LOCAL_REG_COUNT);
  EspModbus.addHreg(LOCAL_REG_START, 0, LOCAL_REG_COUNT);
  EspModbus.addCoil(LOCAL_REG_START, false, LOCAL_REG_COUNT);

  // EspModbus.onSetCoil(LOCAL_REG_MANUALCONTROL, cbWriteCoil, 1);
  // EspModbus.onSetHreg(LOCAL_REG_SSRPOWER, cbWriteHolding, 1);

  for(;;) {
    EspModbus.task();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void otaTask(void *pvParameters) {
  Serial.println("Started otaTask");

  xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
                  ULONG_MAX, /* Reset the notification value to 0 on exit. */
                  NULL,
                  portMAX_DELAY );  /* Block indefinitely. */

  Serial.println("Unblocked otaTask");

  // ArduinoOTA.setPort(3232); 

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("\nAuth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("\nBegin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("\nConnect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("\nReceive Failed");
    else if (error == OTA_END_ERROR) Serial.println("\nEnd Failed");
  });

  Serial.println("OTA Initialized");

  ArduinoOTA.begin();
  for (;;) {
    ArduinoOTA.handle();
    delay(3500);
  }
}

void pwmTask(void *pvParameters) {
  Serial.println("Started pwmTask");

  xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
                  ULONG_MAX, /* Reset the notification value to 0 on exit. */
                  NULL,
                  portMAX_DELAY );  /* Block indefinitely. */

  Serial.println("Unblocked pwmTask once");

  xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
                  ULONG_MAX, /* Reset the notification value to 0 on exit. */
                  NULL,
                  portMAX_DELAY );  /* Block indefinitely. */


  Serial.println("Unblocked pwmTask twice");

  ledcAttachPin(PWM1_GPIO, PWM1_CH);
  ledcSetup(PWM1_CH, PWM1_FREQ, PWM1_RES);
  ledcWrite(PWM1_CH, 0);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  for(;;) {
    if (EspModbus.Coil(LOCAL_REG_MANUALCONTROL)) {
      Serial.println("Manual control enabled, skipping calculations");
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Manual control enabled, skipping calculations");
      }

      ssrPower = EspModbus.Hreg(LOCAL_REG_SSRPOWER);
    } else {
      power1.x = (((unsigned long)EastronModbus.Ireg(12) << 16) | EastronModbus.Ireg(13));
      float p1 = power1.f;
      Serial.print("Current L1 power: ");
      Serial.println(p1);
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Current L1 power: %f", p1);
      }

      power2.x = (((unsigned long)EastronModbus.Ireg(14) << 16) | EastronModbus.Ireg(15));
      float p2 = power2.f;
      Serial.print("Current L2 power: ");
      Serial.println(p2);
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Current L2 power: %f", p2);
      }

      power3.x = (((unsigned long)EastronModbus.Ireg(16) << 16) | EastronModbus.Ireg(17));
      float p3 = power3.f;
      Serial.print("Current L3 power: ");
      Serial.println(p3);
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Current L3 power: %f", p3);
      }

      // let's calculate how much we're exporting to grid or importing
      float_t gridOverflow = p1 + p2 + p3;
      EspModbus.Ireg(LOCAL_REG_GRIDOVERFLOW, int(round(gridOverflow)));

      float gridImport = 0;
      float gridExport = 0;
      if (gridOverflow >= 0) {
        gridImport = round(gridOverflow);
      } else {
        gridExport = round(gridOverflow * -1);
      }

      Serial.print("Current gridImport: ");
      Serial.println(gridImport);
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Current gridImport: %f", gridImport);
      }
      EspModbus.Ireg(LOCAL_REG_GRIDIMPORT, int(gridImport));

      Serial.print("Current gridExport: ");
      Serial.println(gridExport);
      if (WiFi.status() == WL_CONNECTED) {
        syslog.logf(LOG_INFO, "Current gridExport: %f", gridExport);
      }
      EspModbus.Ireg(LOCAL_REG_GRIDEXPORT, int(gridExport));

      // if we're sending something, redirect it to heating body
      if (gridImport > 0) {
        ssrPower -= int(gridImport);
      } else if (gridExport > 0) {
        ssrPower += int(gridExport);
      }
    }

    if (ssrPower > MAX_PWM_POWER) {
      ssrPower = MAX_PWM_POWER;
    }
    if (ssrPower < 0) {
      ssrPower = 0;
    }
    Serial.print("Setting ssrPower to: ");
    Serial.println(ssrPower);
    if (WiFi.status() == WL_CONNECTED) {
      syslog.logf(LOG_INFO, "Setting ssrPower to: %d", ssrPower);
    }
    EspModbus.Ireg(LOCAL_REG_SSRPOWER, ssrPower);

    if (ssrPower < MIN_PWM_POWER) {
      Serial.println("ssrPower too low, setting 0 instead");
      dutyCycle = 0;
      ssrPowerLast = 0;
    } else if (abs(ssrPower - ssrPowerLast) > MIN_PWM_DIFF) {
      Serial.println("Diff in ssrPower higher than threshold, setting new value");
      dutyCycle = calculateDutyCycle(ssrPower);
      ssrPowerLast = ssrPower;
    } else {
      Serial.println("No change to ssrPower");
    }
    Serial.print("Setting ssrPowerLast to: ");
    Serial.println(ssrPowerLast);

    Serial.print("Setting dutyCycle to: ");
    Serial.println(dutyCycle);
    if (WiFi.status() == WL_CONNECTED) {
      syslog.logf(LOG_INFO, "Setting dutyCycle to: %d", dutyCycle);
    }
    EspModbus.Ireg(LOCAL_REG_DUTYCYCLE, dutyCycle);

    ledcWrite(PWM1_CH, dutyCycle);

    vTaskDelay(15000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // serial console
  Serial.begin(115200);

  xTaskCreate(
    infiniTask,
    "InfiniSolar",
    2000,
    NULL,
    1,
    &infiniTaskHandle
  );

  xTaskCreate(
    otaTask,
    "OTA",
    10000,
    NULL,
    1,
    &otaTaskHandle
  );

  xTaskCreate(
    mbServerTask,
    "ModbusTCP",
    5000,
    NULL,
    1,
    &espTaskHandle
  );

  xTaskCreate(
    pwmTask,
    "PWM",
    10000,
    NULL,
    1,
    &pwmTaskHandle
  );

  // must be one but last
  xTaskCreatePinnedToCore(
    keepWifiTask,
    "WifiTask",
    5000,
    NULL,
    1,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );

  // must be last one
  xTaskCreatePinnedToCore(
    eastronTask,
    "Eastron",
    2000,
    NULL,
    2,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );
}

void loop() {}
