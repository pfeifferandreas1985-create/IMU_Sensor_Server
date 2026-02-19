/*
 * ================================================================
 * MAIN + ERWEITERTER WATCHDOG
 * Alle Sensoren aktiv - bei Stuck: Serial zeigt WELCHER Sensor hängt!
 * Beispiel Serial Output bei Fehler:
 *   [ERR] STUCK: MPU6050 (0x68)
 *   [ERR] STUCK: BMP280  (0x77)
 *   [ERR] STUCK: AHT20   (0x38)
 *   [ERR] STUCK: QMC5883P(0x2C)
 * ================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include "AHT20.h"
#include "MadgwickAHRS.h"
#include <qmc5883p.h>
#include "calibration_values.h"

const char* ssid = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";
const char* hostIpAddr = "192.168.178.255";
const int udpPort = 9000;

#define SDA_PIN 22
#define SCL_PIN 21
#define SAMPLE_INTERVAL_US 4000

// I2C Adressen aller Teilnehmer
#define ADDR_MPU6050  0x68
#define ADDR_BMP280   0x77
#define ADDR_AHT20    0x38
#define ADDR_QMC5883P 0x2C

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

void IRAM_ATTR onTimer() { dataReady = true; }

// Prüft ob ein I2C-Teilnehmer antwortet. Gibt true zurück wenn OK.
bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// Vollständiger Watchdog: Prüft alle 4 Sensoren
bool i2cWatchdog() {
    bool ok = true;

    if (!i2cPing(ADDR_MPU6050)) {
        Serial.println("[ERR] STUCK: MPU6050  (0x68) - resetting I2C...");
        ok = false;
    }
    if (!i2cPing(ADDR_BMP280)) {
        Serial.println("[ERR] STUCK: BMP280   (0x77)");
        ok = false;
    }
    if (!i2cPing(ADDR_AHT20)) {
        Serial.println("[ERR] STUCK: AHT20    (0x38)");
        ok = false;
    }
    if (!i2cPing(ADDR_QMC5883P)) {
        Serial.println("[ERR] STUCK: QMC5883P (0x2C)");
        ok = false;
    }

    if (!ok) {
        // I2C Bus Reset
        Wire.end();
        delay(20);
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);
        // Sensoren neu initialisieren
        mpu.initialize();
        bmp.begin(ADDR_BMP280);
        aht.begin();
        mag.begin();
        delay(50);
    }

    return ok;
}

void setup() {
    Serial.begin(115200);
    Serial.println("[MAIN] Start mit erweitertem Watchdog");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\n[OK] WiFi");
    hostIP.fromString(hostIpAddr);

    mpu.initialize();
    bmp.begin(ADDR_BMP280);
    aht.begin();
    if(mag.begin()) Serial.println("[OK] Mag");
    else Serial.println("[WARN] Mag FAIL");

    // I2C Status beim Start ausgeben
    Serial.println("=== I2C Scan beim Start ===");
    Serial.printf("MPU6050  (0x68): %s\n", i2cPing(ADDR_MPU6050)  ? "OK" : "FAIL");
    Serial.printf("BMP280   (0x77): %s\n", i2cPing(ADDR_BMP280)   ? "OK" : "FAIL");
    Serial.printf("AHT20    (0x38): %s\n", i2cPing(ADDR_AHT20)    ? "OK" : "FAIL");
    Serial.printf("QMC5883P (0x2C): %s\n", i2cPing(ADDR_QMC5883P) ? "OK" : "FAIL");
    Serial.println("===========================");

    filter.begin(500.0f);
    filter.setBeta(1.0f);

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_INTERVAL_US, true, 0);
}

void loop() {
    if (dataReady) {
        dataReady = false;

        // Erweiterter Watchdog: prüft ALLE Sensoren
        if (!i2cWatchdog()) {
            return; // Reset durchgeführt, diesen Loop überspringen
        }

        uint32_t ts = micros();
        int16_t gx, gy, gz, ax, ay, az;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float gyro[3] = { gx/131.0f*(PI/180.0f), gy/131.0f*(PI/180.0f), gz/131.0f*(PI/180.0f) };
        float acc[3]  = { ax/16384.0f*9.81f, ay/16384.0f*9.81f, az/16384.0f*9.81f };
        gyro[0] -= GYRO_BIAS[0]; gyro[1] -= GYRO_BIAS[1]; gyro[2] -= GYRO_BIAS[2];

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

        filter.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], m[0], m[1], m[2]);

        SensorPacket pkt; memset(&pkt, 0, sizeof(pkt));
        pkt.timestamp_us = ts;
        memcpy(pkt.gyro, gyro, 12); memcpy(pkt.acc, acc, 12); memcpy(pkt.mag, m, 12);
        float q[4]; filter.getQuaternion(q); memcpy(pkt.quaternion, q, 16);
        memcpy(pkt.angular_velocity, gyro, 12); memcpy(pkt.linear_acceleration, acc, 12);
        pkt.pressure = p; pkt.temp = t; pkt.humidity = h; pkt.mag_yaw = filter.getYaw();

        udp.beginPacket(hostIP, udpPort);
        udp.write((uint8_t*)&pkt, sizeof(pkt));
        udp.endPacket();
    }
}
