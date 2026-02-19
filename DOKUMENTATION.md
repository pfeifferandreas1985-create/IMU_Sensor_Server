# Dokumentation: IMU Sensor Server (GY-271 / QMC5883P Variante)

## Hardware Identifikation: "Fake" GY-271
Es hat sich herausgestellt, dass das verbaute Magnetometer-Modul (beschriftet als GY-271) **nicht** den Standard-Chip QMC5883L (Adresse `0x0D`) oder HMC5883L (Adresse `0x1E`) verwendet.

Stattdessen handelt es sich um den neueren/selteneren **QMC5883P**.

### Merkmale des QMC5883P
*   **I2C Adresse:** `0x2C` (Wichtigstes Unterscheidungsmerkmal)
*   **Register-Map:** Völlig inkompatibel zu L-Versionen.
    *   Status Register: `0x00`
    *   Daten Register: `0x01` - `0x06`
    *   Config Register: `0x20`, `0x21`

### Quellen & Referenzen
*   **Problem-Diskussion:** [Arduino Forum: Are all new GY-270 board magnetometers now fake?](https://forum.arduino.cc/t/are-all-new-gy-270-board-magnetometers-now-fake/1379276)
*   **Datenblatt:** [QMC5883P Datasheet Rev.C](https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-19%20QMC5883P%20Datasheet%20Rev.C(1).pdf)
*   **Treiber-Bibliothek:** [Granddyser/QMC5883P](https://github.com/Granddyser/QMC5883P)

---

## Software Anpassungen

### Verwendete Bibliotheken
Das Projekt nutzt nun die `QMC5883P` Library anstelle der Standard-Treiber.

### Installation
Die Library muss im `libraries` Ordner vorhanden sein:
`git clone https://github.com/Granddyser/QMC5883P.git`

### Code Beispiel (Initialisierung)
```cpp
#include <QMC5883P.h>
QMC5883P mag;

void setup() {
    Wire.begin(21, 22);
    if(mag.begin()) {
        Serial.println("QMC5883P detected @ 0x2C");
    }
}
```
