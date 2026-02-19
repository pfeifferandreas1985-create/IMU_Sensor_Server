# 🤖 IMU Sensor Server — ESP32 Hexapod Sensor Stack

[![ESP32](https://img.shields.io/badge/ESP32-Arduino-blue)](https://www.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

A high-performance 250Hz IMU sensor server running on ESP32, transmitting fused sensor data via UDP to a PC host application for real-time visualization and robotics control.

## 📦 Hardware — Sensors

| Module | Chip | I2C Address | Function |
|--------|------|-------------|----------|
| MPU6050 | InvenSense MPU-6050 | `0x68` | Gyroscope + Accelerometer (250Hz) |
| GY-271 | QMC5883P | `0x2C` | Magnetometer / Compass |
| BMP280 | Bosch BMP280 | `0x77` | Barometric Pressure + Temperature |
| AHT20 | ASAIR AHT20 | `0x38` | Humidity + Temperature |
| ESP32 Dev Module | — | — | WiFi + I2C Controller |

**Pinout:**
- `SDA` → GPIO 22
- `SCL` → GPIO 21
- I2C Bus Speed: 100kHz (Standard)

## 🔧 Software Architecture

```
ESP32 Firmware (250Hz Timer Interrupt)
    ↓ UDP Broadcast (Port 9000)
PC Host App (imu_viewer.py)
    → Real-time 3D Visualization
```

### Firmware Features
- **250Hz sampling** via hardware timer interrupt
- **Madgwick AHRS filter** for quaternion fusion (Gyro + Acc + Mag)
- **Smart I2C throttling**: Mag @20Hz, BMP280 @5Hz, AHT20 @0.5Hz
- **I2C Watchdog** with 9-clock-pulse bus recovery (I2C standard)
- **Brownout protection**: TX Power limited + detector disabled
- **Sim2Real calibration**: Gyro bias and Hard/Soft Iron mag correction

## ⚠️ Known Issues & Fixes Applied

### "Sensor Stuck" / Random Reboots

**Root Cause 1: I2C Watchdog overload**
- Original firmware pinged all sensors every loop (250×/s!)
- Fix: Watchdog runs every 5 seconds only

**Root Cause 2: AHT20 library blocks I2C for 80ms**
- Adafruit AHTX0 `getEvent()` has internal `delay(80)`
- Fix: AHT20 polled every 2000ms instead of 100ms

**Root Cause 3: ESP32 Brownout from WiFi TX peak current**
- WiFi TX at full power (20dBm) draws ~480mA peak → voltage dip → reset
- Fix: `WiFi.setTxPower(WIFI_POWER_11dBm)` + brownout detector disabled
- Hardware fix: 100µF + 100nF decoupling caps on ESP32 VCC/GND

**Root Cause 4: I2C Bus not properly recovered after hang**
- `Wire.end()/begin()` doesn't release a stuck SDA line
- Fix: 9-clock-pulse recovery (official I2C recovery standard)

## 📁 Project Structure

```
IMU_Sensor_Server/
├── firmware_complete/          # Main production firmware
│   ├── firmware_complete.ino   # ESP32 Arduino sketch
│   └── calibration_values.h    # Sensor calibration offsets
├── firmware_debug/             # Debug variants (one sensor disabled each)
│   ├── Variante_A_ohneBMP280/
│   ├── Variante_B_ohneAHT20/
│   ├── Variante_C_ohneQMC5883P/
│   ├── Variante_D_ohneMPU6050/
│   ├── Main_erweiterterWatchdog/
│   └── DeepDiagnostic/         # Full diagnostic with reset reason logging
├── host_app/
│   └── imu_viewer.py           # PC viewer / UDP receiver
├── firmware/                   # Source folder (alternative structure)
├── i2c_scanner/                # I2C bus scanner utility
└── mag_test/                   # Magnetometer test utility
```

## 🚀 Setup & Flash

### Requirements
- Arduino IDE 1.8.x or 2.x
- ESP32 board package 3.x (`esp32` by Espressif)
- Libraries:
  - `MPU6050` (Electronic Cats)
  - `Adafruit BMP280`
  - `Adafruit AHTX0`
  - `Adafruit Unified Sensor`
  - `QMC5883P` (custom)
  - `MadgwickAHRS` (custom)

### Flash
1. Open `firmware_complete/firmware_complete.ino` in Arduino IDE
2. Select board: `ESP32 Dev Module`
3. Set WiFi credentials in the sketch header
4. Flash at 921600 baud

## 🖥️ Host App

```bash
pip install numpy matplotlib scipy
python3 host_app/imu_viewer.py
```

Listens on UDP port 9000 for sensor packets. Displays real-time quaternion data.

## 📝 Calibration

Calibration values are stored in `calibration_values.h`:
- `GYRO_BIAS[3]` — Static gyro drift offset per axis
- `MAG_OFFSET[3]` — Hard-iron offset (compass center shift)
- `MAG_SCALE[3]` — Soft-iron scaling factor per axis

## 📄 License

MIT License — Use freely, attribution appreciated.
