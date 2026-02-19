#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include "AHT20.h"
#include "MadgwickAHRS.h"
#include <qmc5883p.h>
#include "calibration_values.h"
#include "esp_system.h"       // Für Reset-Grund
#include "soc/soc.h"          // Brownout-Detector Disable
#include "soc/rtc_cntl_reg.h" // Brownout-Detector Disable

/* 
 * HEXAPOD SENSOR SERVER V4.3 (Sim2Real Calibration Ready)
 * GENERATED AT: 2026-02-19 01:15 UTC (RESTORED)
 * Pinout: SDA=22, SCL=21 (Swapped!)
 * Sensors: MPU6050, BMP280, AHT20, QMC5883P
 */

const char* ssid = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";
const char* hostIpAddr = "192.168.178.255"; // BROADCAST an alle! (Fixiert Verbindungsprobleme)
const int udpPort = 9000;

#define SDA_PIN 22
#define SCL_PIN 21
#define SAMPLE_INTERVAL_US 4000 // 250 Hz (Reduziert Last auf I2C Bus gegen Hänger)

struct __attribute__((packed)) SensorPacket {
    uint32_t timestamp_us;
    float gyro[3]; float acc[3]; float mag[3];
    float quaternion[4];
    float angular_velocity[3]; float linear_acceleration[3];
    float pressure; float temp; float humidity; float mag_yaw;
};

WiFiUDP udp;
IPAddress hostIP;
MPU6050 mpu;
Adafruit_BMP280 bmp;
AHT20 aht;
QMC5883P mag; 
Madgwick filter;

volatile bool dataReady = false;
hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer() {
    dataReady = true;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    // *** RESET-GRUND: Zeigt warum der ESP32 neugestartet hat ***
    esp_reset_reason_t r = esp_reset_reason();
    const char* reasons[] = {"??","POWER_ON","EXT","SW","PANIC/CRASH","INT_WDT","TASK_WDT","WDT","DEEPSLEEP","BROWNOUT!","SDIO"};
    Serial.printf("[BOOT] Reset-Grund: %s\n", r < 11 ? reasons[r] : "UNBEKANNT");
    if (r == ESP_RST_BROWNOUT) Serial.println("[!] BROWNOUT! Netzteil/Kabel prüfen!");
    if (r == ESP_RST_PANIC)    Serial.println("[!] CRASH! Exception im Code!");
    
    // I2C STABILITÄT: 100kHz + Timeout gegen ewige Hänger
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(3000); // I2C Timeout: 3ms, verhindert ewige Hänger

    // BROWNOUT FIX 1: Brownout-Detektor deaktivieren
    // Verhindert Resets bei kurzen Spannungseinbrüchen (z.B. durch WiFi-Sendepuls)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    // BROWNOUT FIX 2: WiFi TX Leistung reduzieren
    // 20dBm (Standard) = ~480mA Peak → Brownout!
    // 11dBm = ~120mA Peak → stabil. Reichweite im Heimnetz nach wie vor >20m.
    WiFi.setTxPower(WIFI_POWER_11dBm);
    Serial.println("\n[OK] WiFi (TX: 11dBm)");
    hostIP.fromString(hostIpAddr);

    mpu.initialize();
    bmp.begin(0x77);
    aht.begin();
    
    if(mag.begin()) Serial.println("Mag OK"); 
    else Serial.println("Mag FAIL");

    filter.begin(250.0f); // Korrekte Abtastrate (Timer = 250Hz)
    filter.setBeta(1.0f); // Reaktiv ohne Zittern

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_INTERVAL_US, true, 0);
}

// Echter I2C Bus-Recovery: 9 Clock-Pulse befreien eingefrorene SDA-Leitung
// (Offizieller I2C-Standard, deutlich effektiver als Wire.end()/begin())
void i2cBusRecovery() {
    Serial.println("[WDG] I2C Bus Recovery (9-clock)...");
    Wire.end();
    pinMode(SCL_PIN, OUTPUT);
    pinMode(SDA_PIN, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
        digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    }
    // STOP condition
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(3000);
    mpu.initialize(); bmp.begin(0x77); aht.begin(); mag.begin();
    delay(50);
    Serial.println("[WDG] Bus Recovery done");
}

static uint32_t lastWdg = 0;

void doI2cWatchdog() {
    bool ok = true;
    auto ping = [](uint8_t addr) {
        Wire.beginTransmission(addr);
        return Wire.endTransmission() == 0;
    };
    if (!ping(0x68)) { Serial.println("[ERR] MPU6050 (0x68) stuck!"); ok = false; }
    if (!ping(0x77)) { Serial.println("[ERR] BMP280  (0x77) stuck!"); ok = false; }
    if (!ping(0x38)) { Serial.println("[ERR] AHT20   (0x38) stuck!"); ok = false; }
    if (!ping(0x2C)) { Serial.println("[ERR] QMC5883P(0x2C) stuck!"); ok = false; }
    if (!ok) i2cBusRecovery();
}

void loop() {
    if (dataReady) {
        dataReady = false;

        // I2C WATCHDOG: Alle 5000ms (nicht jede Loop!)
        if (millis() - lastWdg > 5000) {
            doI2cWatchdog();
            lastWdg = millis();
        }


        uint32_t ts = micros();

        int16_t gx, gy, gz, ax, ay, az;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float gyro[3] = { (float)gx/131.0f * (float)(PI/180.0f), (float)gy/131.0f * (float)(PI/180.0f), (float)gz/131.0f * (float)(PI/180.0f) };
        float acc[3] = { (float)ax/16384.0f * 9.81f, (float)ay/16384.0f * 9.81f, (float)az/16384.0f * 9.81f };

        // APPLY CALIBRATION (Sim2Real Precision)
        gyro[0] -= GYRO_BIAS[0]; 
        gyro[1] -= GYRO_BIAS[1]; 
        gyro[2] -= GYRO_BIAS[2];

        // Static memory to hold last valid Mag reading
        static float m[3] = {0,0,0};
        
        // MAG THROTTLE: Nur alle 50ms lesen (20 Hz), sonst blockiert der GY-271 den Bus!
        static uint32_t lastMag = 0;
        if(millis() - lastMag > 50) {
            mag.readXYZ(m); 
            // APPLY CALIBRATION (Hard/Soft Iron)
            m[0] = (m[0] - MAG_OFFSET[0]) * MAG_SCALE[0];
            m[1] = (m[1] - MAG_OFFSET[1]) * MAG_SCALE[1];
            m[2] = (m[2] - MAG_OFFSET[2]) * MAG_SCALE[2];
            lastMag = millis();
        }

        static uint32_t lastEnv = 0;
        static uint32_t lastAht = 0;
        static float p=0, t=0, h=0;
        if(millis() - lastEnv > 200) {
            p = bmp.readPressure();
            lastEnv = millis();
        }
        // AHT20 FIX: Adafruit-Library blockiert intern 80ms via delay()!
        // Das friert den ganzen I2C-Bus ein → nur alle 2s lesen reicht völlig.
        if(millis() - lastAht > 2000) {
            aht.getSensor(&h, &t);
            lastAht = millis();
        }

        filter.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], m[0], m[1], m[2]);

        SensorPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.timestamp_us = ts;
        memcpy(pkt.gyro, gyro, 12); 
        memcpy(pkt.acc, acc, 12); 
        memcpy(pkt.mag, m, 12);
        
        float q[4];
        filter.getQuaternion(q);
        memcpy(pkt.quaternion, q, 16);
        
        memcpy(pkt.angular_velocity, gyro, 12); 
        memcpy(pkt.linear_acceleration, acc, 12);
        
        pkt.pressure = p; 
        pkt.temp = t; 
        pkt.humidity = h; 
        pkt.mag_yaw = filter.getYaw();

        udp.beginPacket(hostIP, udpPort);
        udp.write((uint8_t*)&pkt, sizeof(pkt));
        udp.endPacket();
    }
}
