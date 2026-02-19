# ⚡ Ultimativer Verdrahtungsplan V2.3 – Full-MCU Sensor Stack

**Projekt:** Hexapod RL – Sim2Real Sensor-Hub
**Fokus:** Maximale Signalintegrität & EMV-Resistenz
**Hardware:** ESP32 Dual-Core + MPU6050, GY-271, BMP280, AHT20

---

## 1. Spannungsversorgung (Power Architecture)

Wir nutzen das 5V-System des Hexapods. Die Verteilung erfolgt sternförmig vom ESP32 aus, um Rauschkopplung zu minimieren.

| Komponente | Pin (ESP32) | Signal | Spannung | Notiz |
| :--- | :--- | :--- | :--- | :--- |
| **Haupteingang** | **Vin / 5V** | External Power | 5V DC | Vom BEC / Regler |
| **Masse (Main)** | **GND** | System Ground | 0V | Sternpunkt für alle Sensoren |
| **Sensor Power** | **3.3V** | VCC Bus | 3.3V DC | Geregelter Ausgang des ESP32 |

---

## 2. I²C-Bus Spezifikation (400 kHz Fast Mode)

Alle vier Sensoren liegen parallel am Bus. Die Einhaltung der Pull-up Spezifikation ist kritisch für 500 Hz Sampling.

### 🔌 Bus-Mapping
| Signal | ESP32 Pin | Sensor Pins | Widerstand (Pull-up) | Kabelfarbe |
| :--- | :--- | :--- | :--- | :--- |
| **SDA** | **GPIO 21** | Alle SDA Pins | **4.7 kΩ** gegen 3.3V | **Grün** |
| **SCL** | **GPIO 22** | Alle SCL Pins | **4.7 kΩ** gegen 3.3V | **Gelb** |
| **INT** | **GPIO 4** | MPU6050 INT | – | Blau |

---

## 3. Sensor-Detailkonfiguration & Adressierung

| Sensor | Chip | I²C Adresse | Besonderheit | Montage |
| :--- | :--- | :--- | :--- | :--- |
| **MPU6050** | IMU | **0x68** | AD0 auf GND legen | Center of Mass (CoM) |
| **GY-271** | Mag | **0x1E / 0x0D** | HMC5883L/QMC5883L | Weit weg von Motoren |
| **BMP280** | Baro | **0x76 / 0x77** | SDO auf GND/3.3V | Vor Wind geschützt |
| **AHT20** | Klima | **0x38** | Fest codiert | Frei liegend |

---

## 4. Verdrahtungs-Schema (Vollständig)

```text
                +---------------------------------------+
                |             ESP32 (30/38 Pin)         |
                +---------------------------------------+
                   |       |       |       |       |
                 [Vin]   [GND]  [3.3V]  [G21]   [G22]   [G4]
                   |       |       |       |       |      |
  5V (BEC) --------+       |       |       |       |      |
                           |       |       |       |      |
  GND (BEC) ---------------+-------+       |       |      |
                                   |       |       |      |
                                   +-------|-------|------|-----> [3.3V Rail]
                                   |       |       |      |
                                   |    [4.7k]  [4.7k]    |      (Pull-ups an 3.3V)
                                   |       |       |      |
                                   +-------+-------+      |
                                           |       |      |
            +------------------------------+-------+------|------------------+
            |               I2C BUS (SDA / SCL)           |                  |
            |                                             |                  |
      +------------+          +------------+        +------------+     +------------+
      |  MPU6050   |          |   GY-271   |        |   BMP280   |     |   AHT20    |
      +------------+          +------------+        +------------+     +------------+
      | VCC: 3.3V  |          | VCC: 3.3V  |        | VCC: 3.3V  |     | VCC: 3.3V  |
      | GND: GND   |          | GND: GND   |        | GND: GND   |     | GND: GND   |
      | SDA: G21   |          | SDA: G21   |        | SDA: G21   |     | SDA: G21   |
      | SCL: G22   |          | SCL: G22   |        | SCL: G22   |     | SCL: G22   |
      | INT: G4    |          |            |        |            |     |            |
      | AD0: GND   |          |            |        |            |     |            |
      +------------+          +------------+        +------------+     +------------+
```

---

## 5. Engineering Best-Practices (Nichts auslassen!)

### 5.1 Signal-Integrität
*   **Widerstände:** Die 4.7 kΩ Pull-ups müssen so nah wie möglich am ESP32 sitzen.
*   **Leitungsführung:** SDA und SCL sollten **nie** parallel zu Motorstromkabeln laufen. Falls nötig: Verdrille SDA mit einem GND-Draht (Twisted Pair).
*   **Kabellänge:** Absolute Grenze 15-20 cm. Darüber hinaus bricht die Flankensteilheit bei 400 kHz ein.

### 5.2 Filterung & Entstörung
*   **Stützkondensatoren:** Platziere einen **100 nF Keramik-Kondensator** (Code 104) so nah wie möglich an den VCC/GND Pins *jedes* Sensors.
*   **Bulk-Cap:** Ein **10 µF Elektrolyt-Kondensator** an der 5V-Eingangsseite des ESP32 puffert Stromspitzen beim WLAN-Sendeversuch.

### 5.3 Mechanische Montage
*   **Vibrations-Dämpfung:** Die MPU6050 sollte auf einem kleinen Pad aus weichem Neopren oder 3D-gedrucktem TPU gelagert werden, um Motor-Vibrationen (High-Frequency Noise) mechanisch zu filtern.
*   **Barometer-Schutz:** Klebe ein kleines Stück offenporigen Schaumstoff über das BMP280 Gehäuse. Das verhindert "Höhensprünge" durch Luftturbulenzen beim schnellen Laufen des Hexapods.

### 5.4 Adress-Check (Pre-Flight)
Bevor du den Code startest, führe einen I2C-Scanner Sketch aus. Du **musst** folgende Adressen sehen:
1. `0x68` (MPU)
2. `0x1E` oder `0x0D` (Mag)
3. `0x76` oder `0x77` (Baro)
4. `0x38` (AHT)

---

**Status:** Dieser Plan ist final und deckt alle elektrischen und mechanischen Aspekte für einen stabilen 500 Hz Betrieb ab.
