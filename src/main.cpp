#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <Syslog.h>
#include <WiFi.h>

// https://support.innon.com/PowerMeters/SDM630-MOD-MID/Manual/SDM630-Modbus_Protocol.pdf
#define EASTRON_SLAVE_ID 2
#define EASTRON_RX_TX_PIN 21
#define EASTRON_RX_PIN 22
#define EASTRON_TX_PIN 23
#define INFINISOLAR_SLAVE_ID 2
#define INFINISOLAR_RX_TX_PIN 17
#define INFINISOLAR_RX_PIN 18
#define INFINISOLAR_TX_PIN 19
#define EASTRON_NUM_REGS 382

#define WIFI_SSID "smarthome2_IoT"
#define WIFI_PASSWORD "strongpassword"
#define WIFI_TIMEOUT_MS 10000

#define SYSLOG_SERVER "192.168.2.3"
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "ESP32"
#define APP_NAME "EastronMITM"

IPAddress esp_ip(192, 168, 2, 49);
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

BaseType_t xWiFiReturned;
TaskHandle_t xWifiTaskHandle = NULL;

bool cbSomebodyConnected(IPAddress ip) {
  Serial.print("New TCP client connected: ");
  Serial.println(ip);

  int n = ip.toString().length();
  char char_array[n + 1];
  strcpy(char_array, ip.toString().c_str());
  syslog.logf(LOG_INFO, "New TCP client connected: %s", char_array);
  return true;
}

uint16_t cbRead(TRegister* reg, uint16_t val) {
  Serial.print("Read. Reg RAW#: ");
  Serial.println(reg->address.address);
  return val;
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

    int n = WiFi.localIP().toString().length();
    char char_array[n + 1];
    strcpy(char_array, WiFi.localIP().toString().c_str());
    syslog.logf(LOG_INFO, "WiFi connected. IP address: %s", char_array);

    if (eTaskGetState(xWifiTaskHandle) == eSuspended) {
      vTaskResume(xWifiTaskHandle);
    }
  }
}

void eastronTask(void *pvParameters) {
  // Modbus RTU to power meter
  EastronSerial.begin(19200, SERIAL_8N1, EASTRON_RX_PIN, EASTRON_TX_PIN);
  EastronModbus.begin(&EastronSerial, EASTRON_RX_TX_PIN);
  EastronModbus.master();

  for(;;) {
    if (!EastronModbus.slave()) {
      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 0, 0, 80);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 80, 80, 26);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 200, 200, 8);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 224, 224, 46);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      EastronModbus.pullIreg(EASTRON_SLAVE_ID, 334, 334, 48);
      while(EastronModbus.slave()) {
        EastronModbus.task();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void infiniTask(void *pvParameters) {
  // Modbus RTU to inverter
  InfiniSolarSerial.begin(19200, SERIAL_8N1, INFINISOLAR_RX_PIN, INFINISOLAR_TX_PIN);
  InfinisolarModbus.begin(&InfiniSolarSerial, INFINISOLAR_RX_TX_PIN);
  InfinisolarModbus.slave(INFINISOLAR_SLAVE_ID);
  InfinisolarModbus.addIreg(0, 0, EASTRON_NUM_REGS);
  InfinisolarModbus.onGetIreg(0, cbRead, EASTRON_NUM_REGS);

  for(;;) {
    InfinisolarModbus.task();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void espTask(void *pvParameters) {
  // Modbus IP for HA
  EspModbus.onConnect(cbSomebodyConnected);
  EspModbus.server();
  EspModbus.addIreg(0, 0, EASTRON_NUM_REGS);

  for(;;) {
    EspModbus.task();
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
    2000,
    NULL,
    1,
    NULL
  );

  xTaskCreatePinnedToCore(
    eastronTask,
    "Eastron",
    2000,
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
