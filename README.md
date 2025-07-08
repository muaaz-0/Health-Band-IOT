# Health-Band-IOT
# ESP32-Based Health Monitoring System with Google Sheets Integration

This project is an ESP32-based real-time health monitoring system that reads data from multiple sensors (Pulse Oximeter, DHT11 for environment and body temperature, PIR for motion) and displays it on an LCD screen while simultaneously uploading the data to Google Sheets using a Google Apps Script.

## 📟 Features

- Measures **Heart Rate (BPM)** and **SpO2** using MAX30100
- Reads **ambient temperature** and **humidity** using a DHT11
- Reads **body temperature** using a second DHT11 sensor
- Calculates **Heat Index**
- Detects **motion** using a PIR sensor
- Displays real-time data on a 16x2 I2C LCD
- Uploads sensor data to **Google Sheets**
- Uses **FreeRTOS multitasking** for efficient parallel sensor handling

---

## 🛠️ Hardware Requirements

- ESP32 Development Board
- MAX30100 Pulse Oximeter Sensor
- Two DHT11 Sensors (for room and body temperature)
- PIR Motion Sensor
- I2C 16x2 LCD
- Jumper wires, breadboard

---

## 🔌 Wiring Overview

| Sensor | Pin | ESP32 GPIO |
|--------|-----|------------|
| DHT11 (Room) | Data | GPIO 15 |
| DHT11 (Body) | Data | GPIO 25 |
| PIR Sensor | OUT | GPIO 5 |
| LCD (I2C) | SDA/SCL | Default I2C (e.g., GPIO 21/22) |
| MAX30100 | SDA/SCL | Default I2C |

> Make sure to power sensors with 3.3V or 5V as per their specifications.

---

## 📶 Wi-Fi & Cloud Setup

- Set your WiFi credentials in the following lines:
  ```cpp
  const char* ssid = "UAVLAB";
  const char* password = "12345678";
