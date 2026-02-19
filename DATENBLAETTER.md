# 📄 Datenblätter & Sensor-Spezifikationen (Engineering Reference)

## 1. MPU-6050 (InvenSense / TDK)
*   **Kernfunktion:** 6-Achs Motion-Tracking (Gyroscope + Accelerometer).
*   **Spannungsbereich:** 2.375V – 3.46V (Nennspannung 3.3V).
*   **Kommunikation:** I2C (bis 400 kHz), AD0 Pin für Adresswahl.
*   **Auflösung:** 16-bit ADC pro Achse.
*   **Messbereiche (Konfigurierbar):**
    *   Gyro: ±250, ±500, ±1000, ±2000 °/s (genutzt: ±1000°/s).
    *   Accel: ±2, ±4, ±8, ±16 g (genutzt: ±4g).
*   **Features:** Digital Low Pass Filter (DLPF), 1024 Byte FIFO, interner Temperatursensor zur Drift-Kompensation.

## 2. GY-271 / HMC5883L / QMC5883L (Magnetometer)
*   **Funktion:** Digitaler Kompass zur absoluten Yaw-Bestimmung.
*   **Spannung:** 3.0V – 5.0V (Level-Shifter oft integriert, hier 3.3V genutzt).
*   **Sampling-Rate:** Bis zu 75 Hz (genutzt: 50 Hz).
*   **Genauigkeit:** 1° bis 2° Kompass-Präzision nach Hard-Iron Kalibrierung.
*   **Wichtig:** Der HMC5883L und QMC5883L sind Pin-kompatibel, nutzen aber unterschiedliche I2C-Adressen (`0x1E` vs. `0x0D`). Die Firmware muss beide prüfen.

## 3. BMP280 (Bosch Sensortec)
*   **Funktion:** Absoluter Luftdruck & Temperatur.
*   **Präzision:** ±0.12 hPa (entspricht ±1m Höhenauflösung).
*   **Stromaufnahme:** 2.7 µA bei 1 Hz Sampling (extrem effizient).
*   **I2C Adressen:** `0x76` (Standard) oder `0x77` (SDO auf High).
*   **Filterung:** Interner IIR-Filter zur Unterdrückung von kurzzeitigen Druckschwankungen (z.B. durch Hexapod-Schritt-Vibrationen).

## 4. AHT20 (ASAIR)
*   **Funktion:** Digitaler Temperatur- & Luftfeuchtesensor.
*   **Messbereich Temp:** -40°C bis +85°C (Genauigkeit ±0.3°C).
*   **Messbereich Hum:** 0% bis 100% RH (Genauigkeit ±2%).
*   **Ansprechzeit:** < 5 Sekunden.
*   **Besonderheit:** Komplett kalibrierter I2C-Ausgang, ersetzt den fehleranfälligen DHT11/22.

## 5. ESP32-WROOM-32 (Espressif)
*   **CPU:** Xtensa® Dual-Core 32-bit LX6 (bis 240 MHz).
*   **Arbeitsspeicher:** 520 KB SRAM.
*   **Logikpegel:** 3.3V (WICHTIG: Pins sind nicht 5V tolerant!).
*   **Peripherie:** 2x I2C Hardware-Module, 12-bit ADC, integriertes WiFi & Bluetooth.
