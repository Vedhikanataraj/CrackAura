# CrackAura 🏗️
### IoT-Based Concrete Wall Health Monitoring System

CrackAura is an IoT-based prototype designed to monitor concrete wall conditions using sensors and vibration analysis. It aims to help identify potential wall quality issues by collecting sensor data, analyzing vibration responses, and displaying test results through a web dashboard hosted on an ESP32.

## 🎯 Project Objectives

- Monitor wall vibration responses using an MPU6050 accelerometer.
- Measure moisture levels using a soil moisture sensor.
- Detect vibration events using a vibration sensor.
- Analyze captured vibration data using signal processing techniques.
- Display test results, wall condition, and sensor readings through a web interface.
- Provide local monitoring and alerts using connected hardware.

## ✨ Key Features

- **Sensor Monitoring:** Collects accelerometer, moisture, and vibration data.
- **Vibration Analysis:** Captures vibration responses and calculates frequency-related measurements.
- **Wall Testing:** Supports controlled impact/tap-based testing.
- **Web Dashboard:** Provides Home, Test Setup, Run Test, History, Analysis, Reports, and Settings pages.
- **JSON Data Storage:** Saves test results and metadata using the ESP32 LittleFS filesystem.
- **REST-Style API:** Exposes endpoints for live readings, test history, test details, and device settings.
- **Hardware Alerts:** Supports status indicators and relay control.

## 🧰 Technologies Used

| Component | Technology |
|---|---|
| Microcontroller | ESP32 |
| Firmware | Arduino C++ |
| Web interface | HTML, CSS, JavaScript |
| Web server | ESP32 WebServer |
| Filesystem | LittleFS |
| Data format | JSON |
| Sensor interface | I2C and GPIO |
| Signal processing | FFT/frequency analysis |
| Sensor library | Adafruit MPU6050 |

## 🔌 Hardware Components

- ESP32 development board
- MPU6050 accelerometer and gyroscope module
- Soil moisture sensor
- Vibration sensor
- Buzzer or status indicator, if included in the prototype
- Relay modules, if required by the hardware setup
- USB data cable
- Jumper wires and prototype mounting materials

## 🏛️ System Architecture

1. Sensors collect vibration and moisture readings.
2. The ESP32 processes the readings and manages test capture.
3. Captured test data is analyzed and stored as JSON files in LittleFS.
4. The ESP32 hosts the web dashboard and API endpoints.
5. A browser connected to the same accessible network displays live readings and saved test results.

**Data flow:**

`Sensors → ESP32 → Signal Analysis → JSON Storage → Web Dashboard`

## 📡 API Endpoints

The ESP32 firmware provides the following local endpoints:

| Endpoint | Purpose |
|---|---|
| `/` | Open the CrackAura dashboard |
| `/api/live` | Retrieve live sensor readings |
| `/api/status` | Retrieve device status |
| `/api/start` | Start a test |
| `/api/stop` | Stop a test |
| `/api/history` | Retrieve test history |
| `/api/test?id=<TEST_ID>` | Retrieve a saved test as JSON |
| `/api/settings` | Update settings |
| `/api/relay` | Control relay functions |
| `/download` | Access the firmware's download handler |

The available HTTP methods and request parameters depend on the firmware implementation.

## 🚀 Getting Started

### Prerequisites

- Arduino IDE
- ESP32 board package by Espressif Systems