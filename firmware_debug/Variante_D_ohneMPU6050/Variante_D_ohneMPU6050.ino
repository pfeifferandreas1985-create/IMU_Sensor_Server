/*
 * ================================================================
 * DEBUG VARIANTE D: OHNE MPU6050 (kein Gyro/Beschleunigung!)
 * Test: Läuft der Sensor-Stack ohne MPU6050 stabil?
 * Falls JA → MPU6050 (0x68) ist das Problem!
 * ACHTUNG: Madgwick-Filter ist deaktiviert (braucht IMU-Daten).
 *          Nur Umgebungssensoren werden übertragen.
 * ================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// #include "MPU6050.h"           // <-- DEAKTIVIERT: MPU6050
#include <Adafruit_BMP280.h>
#include "AHT20.h"
// #include "MadgwickAHRS.h"      // <-- DEAKTIVIERT: Braucht MPU6050
#include <qmc5883p.h>
#include "calibration_values.h"

const char* ssid = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";
const char* hostIpAddr = "192.168.178.255";
const int udpPort = 9000;

#define SDA_PIN 22
#define SCL_PIN 21
#define SAMPLE_INTERVAL_US 4000

struct __attribute__((packed)) SensorPacket {
    uint32_t timestamp_us;
    float gyro[3]; float acc[3]; float mag[3];
    float quaternion[4];
    float angular_velocity[3]; float linear_acceleration[3];
    float pressure; float temp; float humidity; float mag_yaw;
};

WiFiUDP udp;
IPAddress hostIP;
// MPU6050 mpu;                  // <-- DEAKTIVIERT
Adafruit_BMP280 bmp;
AHT20 aht;
QMC5883P mag;
// Madgwick filter;               // <-- DEAKTIVIERT

volatile bool dataReady = false;
hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer() { dataReady = true; }

void setup() {
    Serial.begin(115200);
    Serial.println("[DEBUG D] Start OHNE MPU6050");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\n[OK] WiFi");
    hostIP.fromString(hostIpAddr);

    // mpu.initialize();          // <-- DEAKTIVIERT
    bmp.begin(0x77);
    aht.begin();
    if(mag.begin()) Serial.println("[OK] Mag");
    else Serial.println("[WARN] Mag FAIL");

    // filter.begin(500.0f);      // <-- DEAKTIVIERT

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_INTERVAL_US, true, 0);
}

void loop() {
    if (dataReady) {
        dataReady = false;

        uint32_t ts = micros();

        // MPU6050 deaktiviert → Gyro/Acc bleiben 0
        float gyro[3] = {0,0,0};
        float acc[3]  = {0,0,0};

        static float m[3] = {0,0,0};
        static uint32_t lastMag = 0;
        if(millis() - lastMag > 50) {
            mag.readXYZ(m);
            m[0] = (m[0] - MAG_OFFSET[0]) * MAG_SCALE[0];
            m[1] = (m[1] - MAG_OFFSET[1]) * MAG_SCALE[1];
            m[2] = (m[2] - MAG_OFFSET[2]) * MAG_SCALE[2];
            lastMag = millis();
        }

        static float p=0, t=0, h=0;
        static uint32_t lastEnv = 0;
        if(millis() - lastEnv > 100) {
            p = bmp.readPressure();
            aht.getSensor(&h, &t);
            lastEnv = millis();
        }

        // Madgwick deaktiviert → Quaternion bleibt 0
        float q[4] = {1,0,0,0};

        SensorPacket pkt; memset(&pkt, 0, sizeof(pkt));
        pkt.timestamp_us = ts;
        memcpy(pkt.gyro, gyro, 12); memcpy(pkt.acc, acc, 12); memcpy(pkt.mag, m, 12);
        memcpy(pkt.quaternion, q, 16);
        memcpy(pkt.angular_velocity, gyro, 12); memcpy(pkt.linear_acceleration, acc, 12);
        pkt.pressure = p; pkt.temp = t; pkt.humidity = h; pkt.mag_yaw = 0;

        udp.beginPacket(hostIP, udpPort);
        udp.write((uint8_t*)&pkt, sizeof(pkt));
        udp.endPacket();
    }
}
