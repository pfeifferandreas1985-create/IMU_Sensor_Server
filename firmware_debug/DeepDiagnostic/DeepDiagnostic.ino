/*
 * ================================================================
 * DEEP DIAGNOSTIC FIRMWARE
 * Gibt beim Start den RESET-GRUND aus (Brownout? WDT? Crash?)
 * Loggt jeden Sensor-Schritt einzeln in Serial.
 * ================================================================
 * BITTE SERIAL MONITOR MIT 115200 BAUD ÖFFNEN!
 * Beispiel-Output bei Brownout: [BOOT] Grund=BROWNOUT (Strom!)
 * Beispiel-Output bei WDT:      [BOOT] Grund=TASK_WDT (Code hängt)
 * Beispiel-Output bei I2C-Hänger: [SENSOR] Lese MPU... (danach nix)
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
#include "esp_system.h"         // Für Reset-Reason
#include "esp_task_wdt.h"       // Hardware Task Watchdog

const char* ssid = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";
const char* hostIpAddr = "192.168.178.255";
const int udpPort = 9000;

#define SDA_PIN 22
#define SCL_PIN 21
#define SAMPLE_INTERVAL_US 4000

// Hardware Watchdog: 10 Sekunden Timeout
// Wenn esp_task_wdt_reset() nicht aufgerufen wird → ESP32 resettet
#define WDT_TIMEOUT_SEC 10

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

// Gibt den Reset-Grund als lesbaren Text aus
const char* getResetReason(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWER_ON (Normal)";
        case ESP_RST_SW:        return "SOFTWARE_RESTART (restart() aufgerufen)";
        case ESP_RST_PANIC:     return "PANIC/CRASH (Exception im Code!)";
        case ESP_RST_INT_WDT:   return "INTERRUPT_WDT (Interrupt hängt zu lang!)";
        case ESP_RST_TASK_WDT:  return "TASK_WDT (Loop hängt, WDT-Timeout)";
        case ESP_RST_WDT:       return "WDT (Anderer Watchdog)";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_WAKEUP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT! *** Spannungseinbruch! Netzteil prüfen! ***";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNBEKANNT";
    }
}

// I2C Ping mit Timeout-Check
bool i2cPing(uint8_t addr, const char* name) {
    Serial.printf("[I2C] Ping %s (0x%02X)... ", name, addr);
    Serial.flush();
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        Serial.println("OK");
        return true;
    } else {
        Serial.printf("FAIL (err=%d)\n", err);
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500); // Kurz warten damit USB-Serial bereit ist
    
    Serial.println("\n\n========================================");
    Serial.println("   DEEP DIAGNOSTIC FIRMWARE");
    Serial.println("========================================");
    
    // ↓↓↓ RESET-GRUND ↓↓↓ - Das ist das wichtigste!
    esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("[BOOT] Reset-Grundursache: %s\n", getResetReason(resetReason));
    Serial.println("========================================\n");
    Serial.flush();

    // Hardware Watchdog aktivieren (10s Timeout)
    esp_task_wdt_config_t wdtCfg = {
        .timeout_ms = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdtCfg);
    esp_task_wdt_add(NULL); // Aktuellen Task überwachen
    Serial.println("[WDT] Hardware Watchdog aktiviert (10s Timeout)");

    // I2C starten
    Serial.println("[I2C] Starte Wire...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Serial.println("[I2C] Wire OK, 100kHz");
    
    // I2C Scan beim Start
    Serial.println("\n--- I2C Bus Scan ---");
    i2cPing(ADDR_MPU6050,  "MPU6050");
    i2cPing(ADDR_BMP280,   "BMP280 ");
    i2cPing(ADDR_AHT20,    "AHT20  ");
    i2cPing(ADDR_QMC5883P, "QMC5883P");
    Serial.println("--------------------\n");

    // WiFi
    Serial.println("[WiFi] Verbinde...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500); Serial.print("."); attempts++;
        esp_task_wdt_reset(); // WDT während WiFi-Verbindung kicken
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] OK! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] FEHLER - kein WiFi!");
    }
    hostIP.fromString(hostIpAddr);

    // Sensoren initialisieren - jeden einzeln loggen
    Serial.println("[INIT] MPU6050...");
    mpu.initialize();
    Serial.printf("[INIT] MPU6050 testConnection: %s\n", mpu.testConnection() ? "OK" : "FAIL");
    esp_task_wdt_reset();

    Serial.println("[INIT] BMP280...");
    if (bmp.begin(ADDR_BMP280)) Serial.println("[INIT] BMP280 OK");
    else Serial.println("[INIT] BMP280 FAIL");
    esp_task_wdt_reset();

    Serial.println("[INIT] AHT20...");
    aht.begin();
    Serial.println("[INIT] AHT20 done");
    esp_task_wdt_reset();

    Serial.println("[INIT] QMC5883P...");
    if (mag.begin()) Serial.println("[INIT] QMC5883P OK");
    else Serial.println("[INIT] QMC5883P FAIL!");
    esp_task_wdt_reset();

    filter.begin(500.0f);
    filter.setBeta(1.0f);

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_INTERVAL_US, true, 0);

    Serial.println("\n[READY] System läuft! Warte auf Sensor-Reads...\n");
    Serial.flush();
}

uint32_t loopCount = 0;

void loop() {
    esp_task_wdt_reset(); // Hardware WDT kicken (wichtig!)

    if (dataReady) {
        dataReady = false;
        loopCount++;

        // Alle 250 Loops (~1 Sekunde) Status ausgeben
        if (loopCount % 250 == 0) {
            Serial.printf("[LOOP] %lu Zyklen OK | Heap: %lu bytes frei\n", 
                          loopCount, esp_get_free_heap_size());
            Serial.flush();
        }

        // I2C Watchdog - alle 5 Sekunden (~1250 Loops)
        if (loopCount % 1250 == 0) {
            Serial.println("[WDG] I2C Status-Check:");
            bool allOk = true;
            allOk &= i2cPing(ADDR_MPU6050,  "MPU6050");
            allOk &= i2cPing(ADDR_BMP280,   "BMP280 ");
            allOk &= i2cPing(ADDR_AHT20,    "AHT20  ");
            allOk &= i2cPing(ADDR_QMC5883P, "QMC5883P");
            if (!allOk) {
                Serial.println("[WDG] Sensor ausgefallen! I2C Reset...");
                Wire.end(); delay(20);
                Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(100000);
                mpu.initialize(); bmp.begin(ADDR_BMP280);
                aht.begin(); mag.begin();
            }
            Serial.flush();
        }

        // ---- Sensor Reads mit Einzellog bei Fehler ----
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
