#include <WiFi.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30100_PulseOximeter.h"
#include "DHT.h"

// WiFi Credentials
const char* ssid = "UAVLAB";
const char* password = "12345678";
const char* scriptURL = "https://script.google.com/macros/s/AKfycbx-LQvMtueizIFhciQSfmzjPTHlo8cRAR2eMDg3VR8RGqrK5fD4_QgjeYPxkvwUvpdH/exec";

// DHT Settings
#define DHTPIN 15
#define BODY_DHTPIN 25
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
DHT bodyDHT(BODY_DHTPIN, DHTTYPE);

float temperature = 0, humidity = 0;
float bodyTemp = 0;
float heatIndex = 0;   // <--- Added Heat Index variable

// Pulse Oximeter
PulseOximeter pox;
float bpm = 0, spo2 = 0;

// PIR Sensor
#define PIR_PIN 5
bool motionDetected = false;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
int lcdPage = 0;  // 0,1,2 for 3 screens

// Mutex for shared data
SemaphoreHandle_t dataMutex;

void onBeatDetected() {
  // optional: LED blink or debug
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(100);
  }
}

// Heat Index calculation function
float calculateHeatIndex(float T, float h) {
  float HI = -8.784695 + 1.61139411 * T + 2.338549 * h - 0.14611605 * T * h
             - 0.012308094 * T * T - 0.016424828 * h * h
             + 0.002211732 * T * T * h + 0.00072546 * T * h * h
             - 0.000003582 * T * T * h * h;
  return HI;
}

// ================== RTOS TASKS =====================

int invalidCount = 0;

void pulseTask(void *pvParameters) {
  while (true) {
    pox.update();

    float newBpm = pox.getHeartRate();
    float newSpo2 = pox.getSpO2();

    // Check if values are valid
    bool valid = (newBpm > 30 && newBpm < 200) && (newSpo2 > 70 && newSpo2 <= 100);

    if (valid) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        bpm = newBpm;
        spo2 = newSpo2;
        xSemaphoreGive(dataMutex);
      }
      invalidCount = 0; // Reset invalid count
    } else {
      invalidCount++;
    }

    // If we get 10+ invalid readings (~1s), treat it as "no finger"
    if (invalidCount >= 10) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        bpm = 0;
        spo2 = 0;
        xSemaphoreGive(dataMutex);
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Run every 100ms
  }
}

void sensorTask(void *pvParameters) {
  unsigned long lastDHT = 0;
  unsigned long lastLCD = 0;
  unsigned long lastUpload = 0;

  while (true) {
    unsigned long now = millis();

    // Motion sensor
    motionDetected = digitalRead(PIR_PIN);

    // DHT every 5s
    if (now - lastDHT > 5000) {
      lastDHT = now;
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (!isnan(t)) temperature = t;
      if (!isnan(h)) humidity = h;

      // Calculate heat index here
      if (!isnan(temperature) && !isnan(humidity)) {
        heatIndex = calculateHeatIndex(temperature, humidity);
      }
    }

    // LCD update every 2s (no HI display, unchanged)
    if (now - lastLCD > 2000) {
      lastLCD = now;
      lcd.clear();
      lcd.setCursor(0, 0);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        if (lcdPage == 0) {
          lcd.print("HR:");
          lcd.print((int)bpm);
          lcd.print(" SpO2:");
          lcd.print((int)spo2);
        } else if (lcdPage == 1) {
          lcd.print("T:");
          lcd.print((int)temperature);
          lcd.print("C H:");
          lcd.print((int)humidity);
        } else if (lcdPage == 2) {
          lcd.print("BTmp:");
          lcd.print((int)bodyTemp);
          lcd.print("C");
        }
        xSemaphoreGive(dataMutex);
      }

      lcd.setCursor(0, 1);
      lcd.print("Motion:");
      lcd.print(motionDetected ? "Yes" : "No ");

      lcdPage = (lcdPage + 1) % 3;
    }

    // Upload every 10s
    if (now - lastUpload > 10000 && WiFi.status() == WL_CONNECTED) {
      lastUpload = now;
      String url;

      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        url = String(scriptURL) +
              "?sts=write" +
              "&hr=" + String((int)bpm) +
              "&spo2=" + String((int)spo2) +
              "&temp=" + String(temperature, 1) +
              "&bt=" + String(bodyTemp, 1) +
              "&humd=" + String(humidity, 0) +
              "&hi=" + String(heatIndex, 1) +       // Added heat index param here
              "&motion=" + (motionDetected ? "Det" : "Ndet");
        xSemaphoreGive(dataMutex);
      }

      HTTPClient http;
      http.begin(url);
      int httpCode = http.GET();
      http.end();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Run every 100ms
  }
}

// New task to read body temperature from second DHT11 (pin 25)
void bodyTempTask(void *pvParameters) {
  unsigned long lastRead = 0;
  while (true) {
    unsigned long now = millis();

    if (now - lastRead > 5000) { // every 5s
      lastRead = now;
      float bt = bodyDHT.readTemperature();
      if (!isnan(bt)) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          bodyTemp = bt;
          xSemaphoreGive(dataMutex);
        }
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ================== SETUP =====================

void setup() {
  Serial.begin(115200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  connectWiFi();
  lcd.clear();
  lcd.print(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi Fail");
  delay(1000);
  lcd.clear();

  // Sensor init
  dht.begin();
  bodyDHT.begin();
  pinMode(PIR_PIN, INPUT);

  if (!pox.begin()) {
    lcd.print("POX Fail");
    while (1); // halt
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Mutex
  dataMutex = xSemaphoreCreateMutex();

  // Create RTOS Tasks
  xTaskCreatePinnedToCore(pulseTask, "PulseTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bodyTempTask, "BodyTempTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Nothing needed here because of tasks
}
