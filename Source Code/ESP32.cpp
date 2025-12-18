#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

// ================= WIFI =================
#define WIFI_SSID     "Hotspot"
#define WIFI_PASSWORD "password"

// ================= FIREBASE =================
#define DATABASE_URL "https://power-monitoring-and-billing-default-rtdb.firebaseio.com/"
#define DATABASE_SECRET "24TVbWhqMmP3HZpSxzV9FDthDwpyqLpzFLBmDKvL"

// ================= FIREBASE OBJECTS =================
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ================= ENERGY PARAMETERS =================
float vrms = 0.0;
float irms = 0.0;
float power = 0.0;
float energy_kWh = 0.0;
float bill = 0.0;

float powerFactor = 0.9;
float costPerUnit = 6.5;

unsigned long lastCalcTime = 0;

// ================= RTOS =================
SemaphoreHandle_t dataMutex;

// ================= UART PARSER =================
bool parsePacket(String pkt)
{
  int v = pkt.indexOf("VRMS=");
  int i = pkt.indexOf("IRMS=");

  if (v < 0 || i < 0) return false;

  float v_val = pkt.substring(v + 5, pkt.indexOf(",", v)).toFloat();
  float i_val = pkt.substring(i + 5, pkt.indexOf(">")).toFloat();

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  vrms = v_val;
  irms = i_val;
  xSemaphoreGive(dataMutex);

  return true;
}

// ================= ENERGY CALC =================
void calculateEnergy()
{
  unsigned long now = millis();
  if (lastCalcTime == 0) {
    lastCalcTime = now;
    return;
  }

  float dt = (now - lastCalcTime) / 3600000.0;
  lastCalcTime = now;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  power = vrms * irms * powerFactor;
  energy_kWh += power * dt / 1000.0;
  bill = energy_kWh * costPerUnit;
  xSemaphoreGive(dataMutex);
}

// ================= SERIAL PRINT =================
void printToSerial()
{
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  Serial.println("===== ENERGY MONITOR =====");
  Serial.printf("Voltage   : %.2f V\n", vrms);
  Serial.printf("Current   : %.4f A\n", irms);
  Serial.printf("Power     : %.2f W\n", power);
  Serial.printf("Energy    : %.6f kWh\n", energy_kWh);
  Serial.printf("Bill      : ₿%.2f\n", bill);
  Serial.println("==========================\n");

  xSemaphoreGive(dataMutex);
}

// ================= FIREBASE UPLOAD =================
void uploadToCloud()
{
  FirebaseJson json;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  json.set("voltage_V_x100", (int)(vrms * 100));
  json.set("current_mA", (int)(irms * 1000));
  json.set("power_W_x10", (int)(power * 10));
  json.set("energy_Wh", (int)(energy_kWh * 1000));
  json.set("bill_x100", (int)(bill * 100));
  xSemaphoreGive(dataMutex);

  if (Firebase.setJSON(fbdo, "/energy_meter/live", json)) {
    Serial.println("Firebase upload OK");
  } else {
    Serial.println(fbdo.errorReason());
  }
}

// ================= TASKS =================

// UART RECEIVE TASK (Core 1)
void uartTask(void *pv)
{
  String rx = "";

  for (;;) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '<') rx = "";
      rx += c;
      if (c == '>') {
        parsePacket(rx);
        rx = "";
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ENERGY CALC TASK (Core 1)
void energyTask(void *pv)
{
  for (;;) {
    calculateEnergy();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// CLOUD TASK (Core 0)
void cloudTask(void *pv)
{
  for (;;) {
    uploadToCloud();
    printToSerial();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  dataMutex = xSemaphoreCreateMutex();

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Firebase
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, a&auth);
  Firebase.reconnectWiFi(true);

  // ================= TASK CREATION =================
  xTaskCreatePinnedToCore(uartTask,   "UART Task",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(energyTask, "Energy Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(cloudTask,  "Cloud Task",  8192, NULL, 1, NULL, 0);

  Serial.println("ESP32 Energy Monitor (FreeRTOS)");
}

// ================= LOOP =================
void loop()
{
  // Empty – FreeRTOS handles everything
}
