#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <WiFi.h>

// https://support.innon.com/PowerMeters/SDM630-MOD-MID/Manual/SDM630-Modbus_Protocol.pdf
#define EASTRON_SLAVE_ID 2
#define INFINISOLAR_SLAVE_ID 2
#define EASTRON_NUM_REGS 382

#define WIFI_SSID "smarthome2_IoT"
#define WIFI_PASSWORD "strongpassword"
#define WIFI_TIMEOUT_MS 10000

IPAddress esp_ip(192, 168, 2, 49);
IPAddress esp_mask(255, 255, 255, 0);
IPAddress esp_gw(192, 168, 2, 1);
IPAddress esp_dns1(192, 168, 2, 2);

HardwareSerial EastronSerial(1);
HardwareSerial InfiniSolarSerial(2);

ModbusRTU EastronModbus;
ModbusRTU InfinisolarModbus;
ModbusIP EspModbus;

BaseType_t xWiFiReturned;
TaskHandle_t xWifiTaskHandle = NULL;

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  }
  return true;
}

bool cbSomebodyConnected(IPAddress ip) {
  Serial.print("New TCP client connected: ");
  Serial.println(ip);
  return true;
}

void keepWifiTask(void *pvParameters) {
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

    if (eTaskGetState(xWifiTaskHandle) == eSuspended) {
      vTaskResume(xWifiTaskHandle);
    }
  }
}

void eastronTask(void *pvParameters) {
  // Modbus RTU to power meter
  EastronSerial.begin(19200, SERIAL_8N1, 22, 23);
  EastronModbus.begin(&EastronSerial);
  EastronModbus.master();

  for(;;) {
    if (!EastronModbus.slave()) {
      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 0, 0, 80, cb);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        delay(10);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 80, 80, 26, cb);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        delay(10);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 200, 200, 8, cb);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        delay(10);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 224, 224, 46, cb);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        delay(10);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 334, 334, 48, cb);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        delay(10);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void infiniTask(void *pvParameters) {
  // Modbus RTU to inverter
  InfiniSolarSerial.begin(19200, SERIAL_8N1, 18, 19);
  InfinisolarModbus.begin(&InfiniSolarSerial);
  InfinisolarModbus.slave(INFINISOLAR_SLAVE_ID);
  InfinisolarModbus.addIreg(0, 0, EASTRON_NUM_REGS);

  for(;;) {
    InfinisolarModbus.task();
    delay(10);
  }
}

void espTask(void *pvParameters) {
  // Modbus IP for HA
  EspModbus.onConnect(cbSomebodyConnected);
  EspModbus.server();
  EspModbus.addIreg(0, 0, EASTRON_NUM_REGS);

  for(;;) {
    EspModbus.task();
    delay(10);
  }
}

void setup() {
  // serial console
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    keepWifiTask,
    "WifiTask",
    5000,
    NULL,
    1,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );

  xTaskCreate(
    infiniTask,
    "InfiniSolar",
    1000,
    NULL,
    1,
    NULL
  );

  xTaskCreatePinnedToCore(
    eastronTask,
    "Eastron",
    1000,
    NULL,
    2,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );

  xWiFiReturned = xTaskCreate(
    espTask,
    "ModbusTCP",
    5000,
    NULL,
    1,
    &xWifiTaskHandle
  );

  if (xWiFiReturned == pdPASS) {
    vTaskSuspend(xWifiTaskHandle);
  }
}

void loop() {}
