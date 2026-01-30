# 🌾 AGRICULTURAL DRONE - COMPLETE SETUP GUIDE

## 📋 Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Wiring Diagrams](#wiring-diagrams)
3. [Software Installation](#software-installation)
4. [System Configuration](#system-configuration)
5. [Deployment Guide](#deployment-guide)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 Hardware Requirements

### Drone Components
| Component | Specification | Quantity |
|-----------|--------------|----------|
| Quadcopter Frame | 450mm - 550mm | 1 |
| Brushless Motors | 1000KV - 1400KV (2212) | 4 |
| ESC (Electronic Speed Controller) | 30A | 4 |
| Flight Controller | Pixhawk 2.4.8 / F4 / KK2.1.5 | 1 |
| Propellers | 10x4.5 / 1045 | 4 pairs |
| Li-Po Battery | 11.1V 3S 2200mAh - 5000mAh | 1-2 |
| Battery Charger | LiPo Balance Charger | 1 |
| Remote Control | 6+ Channel 2.4GHz | 1 |
| Receiver | Compatible with RC | 1 |

### Sensor Module Components
| Component | Specification | Quantity | Pin Type |
|-----------|--------------|----------|----------|
| ESP32 DevKit | 30-pin or 38-pin | 1 | Digital/Analog |
| DHT22 | Temperature & Humidity | 1 | Digital (3-pin) |
| Soil Moisture Sensor | Capacitive or Resistive | 1 | Analog |
| GPS Module | NEO-6M / NEO-M8N | 1 | UART |
| ESP32-CAM | OV2640 Camera | 1 | - |
| Voltage Regulator | 5V Step-down (for sensors) | 1 | - |
| MicroSD Card | 4GB+ (optional) | 1 | - |
| Jumper Wires | Male-Female, Female-Female | 20+ | - |
| Breadboard | 830 points (optional) | 1 | - |

### Power Distribution
- **Main Battery**: Powers motors via ESCs
- **5V BEC**: From ESCs or separate module for flight controller
- **ESP32**: Can run on 3.3V (USB) or 5V (via voltage regulator)
- **Sensors**: 3.3V or 5V depending on module

---

## 🔌 Wiring Diagrams

### ESP32 Pin Configuration

```
ESP32 DevKit (30-pin layout)
========================================

                    ┌─────────┐
                3V3 │1      30│ GND
                 EN │2      29│ GPIO23 (VSYNC)
            GPIO36  │3      28│ GPIO22 (PCLK)
            GPIO39  │4      27│ GPIO1 (TX)
            GPIO34* │5      26│ GPIO3 (RX)
            GPIO35  │6      25│ GPIO21 (Y5)
            GPIO32  │7      24│ GND
            GPIO33  │8      23│ GPIO19 (Y4)
            GPIO25  │9      22│ GPIO18 (Y3)
            GPIO26  │10     21│ GPIO5 (Y2)
            GPIO27  │11     20│ GPIO17 (GPS_TX)
   DHT22 → GPIO14  │12     19│ GPIO16 (GPS_RX)
            GPIO12  │13     18│ GPIO4
                GND │14     17│ GPIO2
            GPIO13  │15     16│ GPIO15
                    └─────────┘

* GPIO34 = Soil Moisture Sensor (ADC1_6)
```

### Complete Wiring Table

#### DHT22 Temperature & Humidity Sensor
```
DHT22          →    ESP32
━━━━━━━━━━━━━━━━━━━━━━━━━
VCC (+)        →    3.3V
DATA           →    GPIO15
GND (-)        →    GND

Note: Add 10kΩ pull-up resistor between VCC and DATA
```

#### Soil Moisture Sensor (Capacitive)
```
Sensor         →    ESP32
━━━━━━━━━━━━━━━━━━━━━━━━━
VCC (+)        →    3.3V
AOUT           →    GPIO34 (ADC1_6)
GND (-)        →    GND

Calibration:
- In air (dry): ~4095
- In water (wet): ~1000-1500
```

#### GPS Module (NEO-6M)
```
GPS Module     →    ESP32
━━━━━━━━━━━━━━━━━━━━━━━━━
VCC            →    3.3V or 5V*
TX             →    GPIO16 (RX2)
RX             →    GPIO17 (TX2)
GND            →    GND

* Check your GPS module voltage requirements
```

#### ESP32-CAM Connections
```
ESP32-CAM is a separate module with integrated camera.
Use separate ESP32-CAM or connect external camera to main ESP32.

For ESP32-CAM pinout, see code comments in drone_sensor_module.ino
Power: 5V (minimum 500mA current capability)
```

### Power Supply Schematic

```
                    LiPo Battery (11.1V)
                           |
                    ┌──────┴──────┐
                    │             │
                Flight         BEC/Voltage
              Controller      Regulator
                (5V)            (5V)
                    │             │
                    │      ┌──────┴──────┐
                    │      │             │
                    │   ESP32         Sensors
                    │   (3.3V)      (3.3V/5V)
                    │
              Motors + ESCs
```

### Complete Assembly Diagram

```
                    ╔═══════════════╗
                    ║   QUADCOPTER  ║
                    ║     FRAME     ║
                    ╚═══════════════╝
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐       ┌─────▼─────┐      ┌────▼────┐
   │ Motor 1 │       │  Flight   │      │ Motor 2 │
   │ + ESC   │       │Controller │      │ + ESC   │
   └─────────┘       │ (Pixhawk) │      └─────────┘
                     └─────┬─────┘
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐       ┌─────▼─────┐      ┌────▼────┐
   │ Motor 3 │       │   ESP32   │      │ Motor 4 │
   │ + ESC   │       │  Sensor   │      │ + ESC   │
   └─────────┘       │  Module   │      └─────────┘
                     └─────┬─────┘
           ┌───────────────┼───────────────┐
           │               │               │
      ┌────▼────┐     ┌────▼────┐     ┌───▼────┐
      │  DHT22  │     │   GPS   │     │  Soil  │
      │ Sensor  │     │ Module  │     │Moisture│
      └─────────┘     └─────────┘     └────────┘
                           │
                     ┌─────▼─────┐
                     │ ESP32-CAM │
                     │  Camera   │
                     └───────────┘
```

---

## 💻 Software Installation

### 1. Arduino IDE Setup

```bash
# Download Arduino IDE from https://www.arduino.cc/en/software

# Add ESP32 Board Support:
# File → Preferences → Additional Board Manager URLs:
https://dl.espressif.com/dl/package_esp32_index.json

# Install ESP32 Board:
# Tools → Board → Boards Manager → Search "ESP32" → Install
```

### 2. Required Arduino Libraries

Install via Tools → Manage Libraries:

```
✓ DHT sensor library (by Adafruit)
✓ Adafruit Unified Sensor
✓ TinyGPS++
✓ ArduinoJson (version 6.x)
✓ ESP32 Camera (built-in with ESP32 board)
```

### 3. Ground Station Setup (Python)

```bash
# Install Python 3.8+
# Download from https://www.python.org/

# Install required packages:
pip install opencv-python numpy matplotlib requests pillow flask pandas

# Or use requirements.txt:
pip install -r requirements.txt
```

### 4. Mobile App Setup (React Native)

```bash
# Install Node.js and npm
# Download from https://nodejs.org/

# Install React Native CLI
npm install -g react-native-cli

# Create project
npx react-native init AgriDroneApp

# Install dependencies
cd AgriDroneApp
npm install @react-navigation/native @react-navigation/stack
npm install axios react-native-maps react-native-chart-kit
npm install react-native-gesture-handler react-native-reanimated

# Copy mobile_app.js to App.js
cp mobile_app.js App.js

# Run on Android
npx react-native run-android

# Run on iOS
npx react-native run-ios
```

---

## ⚙️ System Configuration

### Step 1: Upload ESP32 Code

1. Open `drone_sensor_module.ino` in Arduino IDE
2. Select board: Tools → Board → ESP32 Dev Module
3. Configure settings:
   - Upload Speed: 115200
   - Flash Frequency: 80MHz
   - Flash Mode: QIO
   - Flash Size: 4MB
   - Partition Scheme: Default
4. Select COM Port: Tools → Port → COMx (Windows) or /dev/ttyUSBx (Linux)
5. Click Upload button

### Step 2: WiFi Configuration

```cpp
// In drone_sensor_module.ino, modify these lines:

// For Access Point mode (standalone in field):
const char* ap_ssid = "AgriDrone_AP";        // Change to your preference
const char* ap_password = "AgriDrone2024";   // Change to strong password

// For Station mode (connect to existing WiFi):
const char* sta_ssid = "YOUR_WIFI_SSID";
const char* sta_password = "YOUR_WIFI_PASSWORD";
```

### Step 3: Sensor Calibration

#### Soil Moisture Calibration:
```cpp
// Test in dry air:
// Note the reading (should be ~4095)

// Test in water:
// Note the reading (should be ~1000-1500)

// Update map() function in readSoilMoisture():
currentData.soilMoisture = map(rawValue, 4095, 1000, 0, 100);
//                                        ^^^^  ^^^^
//                                        dry   wet
```

#### DHT22 Testing:
```cpp
// Run a simple test sketch:
#include <DHT.h>
DHT dht(15, DHT22);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  Serial.printf("Temp: %.1f°C, Humidity: %.1f%%\n", temp, hum);
  delay(2000);
}
```

---

## 🚀 Deployment Guide

### Field Deployment Checklist

#### Pre-Flight Checks:
- [ ] Fully charge LiPo battery
- [ ] Check all propellers for damage
- [ ] Verify all sensor connections
- [ ] Test ESP32 WiFi connection
- [ ] Verify GPS lock (blue LED blinking on GPS module)
- [ ] Test camera capture
- [ ] Check flight controller calibration
- [ ] Clear flight area of obstacles

#### Ground Station Setup:
1. Power on drone
2. Connect laptop/phone to drone WiFi: "AgriDrone_AP"
3. Open web browser to http://192.168.4.1
4. Verify sensor readings
5. Test camera feed

#### Flight Operation:
1. Arm flight controller
2. Hover at 2-3 meters initially
3. Gradually increase altitude to 5-10 meters
4. Fly in grid pattern over field
5. Monitor sensor data continuously
6. Capture images at regular intervals
7. Land safely when complete

### Data Collection Strategy

```
Field Grid Pattern (Recommended):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Start →  ═══════════════════╗
                            ║
         ═══════════════════╝
         ╔═══════════════════
         ║
         ╚═══════════════════╗
                            ║
         ═══════════════════╝  ← End

Flight Parameters:
- Altitude: 5-10 meters
- Speed: 2-5 m/s
- Image capture: Every 5 seconds
- Sensor logging: Continuous (1Hz)
```

---

## 🔧 Troubleshooting

### Common Issues and Solutions

#### 1. ESP32 Not Connecting to WiFi
```
Problem: Cannot connect to drone WiFi
Solutions:
✓ Verify ESP32 is powered on (check LED)
✓ Check if "AgriDrone_AP" appears in WiFi list
✓ Confirm password is correct
✓ Move closer to drone (within 50m)
✓ Restart ESP32
```

#### 2. Sensor Reading Errors
```
Problem: DHT22 shows -999 values
Solutions:
✓ Check wiring (VCC, GND, DATA)
✓ Add 10kΩ pull-up resistor
✓ Replace DHT22 sensor
✓ Change GPIO pin and update code

Problem: Soil moisture always shows 0 or 100
Solutions:
✓ Calibrate sensor (see configuration section)
✓ Check analog pin (must use ADC1, not ADC2)
✓ Verify 3.3V power supply
```

#### 3. GPS Not Getting Fix
```
Problem: GPS shows 0 satellites
Solutions:
✓ Move to open sky area (away from buildings)
✓ Wait 2-5 minutes for initial fix
✓ Check GPS antenna placement
✓ Verify TX/RX connections not swapped
✓ Check baud rate (9600 for NEO-6M)
```

#### 4. Camera Not Working
```
Problem: Camera capture fails
Solutions:
✓ Use adequate power supply (5V, >500mA)
✓ Check all camera pin connections
✓ Verify camera model matches code
✓ Reset ESP32-CAM before upload
✓ Use lower image resolution (FRAMESIZE_SVGA)
```

#### 5. Flight Controller Issues
```
Problem: Motors not spinning
Solutions:
✓ Check ESC connections
✓ Calibrate ESCs
✓ Verify transmitter binding
✓ Check flight mode (not in safe mode)
✓ Ensure battery is fully charged
```

### Debug Serial Monitor

```cpp
// Enable detailed debugging:
Serial.begin(115200);
Serial.setDebugOutput(true);

// Monitor output:
// Tools → Serial Monitor (115200 baud)
```

### Error Codes

| Code | Meaning | Solution |
|------|---------|----------|
| E001 | DHT Sensor Failed | Check wiring, replace sensor |
| E002 | Camera Init Failed | Check power, verify pins |
| E003 | WiFi Connection Failed | Check credentials, restart |
| E004 | GPS Timeout | Move to open area, wait longer |
| E005 | Low Battery | Replace/charge battery |

---

## 📊 Performance Optimization

### Battery Life Optimization
- Use efficient flight patterns (grid vs. random)
- Reduce camera resolution for longer flights
- Lower sensor polling frequency
- Use sleep modes when stationary

### Data Quality Tips
- Fly at consistent altitude (±1m)
- Maintain steady speed
- Avoid windy conditions (>15 km/h)
- Calibrate sensors weekly
- Clean camera lens before each flight

### Coverage Calculation

```
Field Coverage Formula:
━━━━━━━━━━━━━━━━━━━━━━
Coverage Area = (Camera FOV × Altitude) × Flight Distance

Example:
Camera FOV: 60°
Altitude: 10m
→ Ground coverage width: ~11m

For 1 hectare (100m × 100m):
Required flight paths: ~9-10 parallel lines
Estimated time: 15-20 minutes
```

---

## 🔒 Safety Guidelines

### Operational Safety
1. **Always maintain visual line of sight**
2. **Do not fly near people or animals**
3. **Check weather conditions (no rain/strong wind)**
4. **Follow local drone regulations**
5. **Keep spare propellers and batteries**
6. **Have fire extinguisher nearby (for LiPo fires)**
7. **Never leave batteries charging unattended**

### Electrical Safety
- Use LiPo safety bag for charging
- Never over-discharge batteries (<3.3V per cell)
- Disconnect battery when not in use
- Check for damaged wires regularly
- Use proper gauge wires for power distribution

---

## 📞 Support and Resources

### Official Documentation
- ESP32: https://docs.espressif.com/
- Arduino: https://www.arduino.cc/reference/
- React Native: https://reactnative.dev/

### Community Forums
- ESP32 Forum: https://esp32.com/
- ArduPilot: https://discuss.ardupilot.org/
- Agriculture Tech: https://www.precisionag.com/

### Video Tutorials
Search YouTube for:
- "ESP32 sensor interfacing"
- "Drone assembly tutorial"
- "Agricultural drone setup"

---

## 📝 Maintenance Schedule

| Task | Frequency |
|------|-----------|
| Clean camera lens | Before each flight |
| Check propellers | Before each flight |
| Calibrate sensors | Weekly |
| Update software | Monthly |
| Replace propellers | After crashes or every 3 months |
| Battery health check | Monthly |
| Tighten screws | Every 10 flights |

---

## 🎓 Next Steps

1. **Start Small**: Test in small area first
2. **Learn Flight Controls**: Practice with remote control
3. **Understand Data**: Learn to interpret sensor readings
4. **Expand Coverage**: Gradually increase field size
5. **Integrate AI**: Add machine learning for disease detection
6. **Automate Flights**: Implement GPS waypoint navigation

---

**Version**: 1.0  
**Last Updated**: January 2026  
**License**: Open Source (MIT)

For questions or support, refer to the troubleshooting section or community forums.

Happy Farming! 🌾✨
