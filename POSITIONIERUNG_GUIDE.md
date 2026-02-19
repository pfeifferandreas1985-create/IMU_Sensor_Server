# 📐 Mechanisches Layout & Positionierung (CoM-Strategie)

Für ein stabiles RL-Training in NVIDIA Isaac Sim ist die physikalische Übereinstimmung von Hardware und Simulation entscheidend. Hier ist die strategische Anordnung der Module im Gehäuse nach **First Principles**:

---

## 1. Das kritische Zentrum: Die MPU6050 (IMU)
Die **MPU6050 MUSS zwingend so nah wie möglich an den Masseschwerpunkt (Center of Mass, CoM)** des Hexapods.

### Warum?
*   **Beschleunigung (Acc):** Wenn die IMU außerhalb des CoM sitzt, misst sie bei jeder Drehung des Roboters zusätzliche Zentripetalbeschleunigungen. In der Simulation wird die Beschleunigung meist direkt im CoM berechnet. Jede Abweichung führt zu "Phantom-Kräften" im RL-State.
*   **Rotation (Gyro):** Die Winkelgeschwindigkeit ist zwar überall am starren Körper gleich, aber die Integration zu Quaternionen ist im Drehzentrum am stabilsten gegen Vibrationen.
*   **Achsen-Ausrichtung:** Die X-Achse des Moduls muss exakt parallel zur Vorwärts-Laufrichtung des Hexapods ausgerichtet sein.

---

## 2. Magnetometer (GY-271): Die "EMV-Distanz"
Das Magnetometer sollte **NICHT** in den Masseschwerpunkt, wenn dort Stromkabel oder Motoren sind.

### Strategie:
*   **Distanz:** Platziere es so weit wie möglich weg von:
    1.  Den Servomotoren (Permanentmagnete & Spulen).
    2.  Den 5V/Batterie-Stromkabeln (Magnetfelder bei Lastspitzen).
*   **Position:** Oft ist eine erhöhte Position (ein kleiner "Turm") oder ein Ausleger am hinteren/vorderen Ende des Chassis ideal.
*   **Wichtig:** Die Ausrichtung muss parallel zur IMU sein, damit die Fusion im Madgwick-Filter mathematisch korrekt bleibt.

---

## 3. Barometer (BMP280): Die "Beruhigungszone"
Der BMP280 misst Luftdruck zur Höhenbestimmung. Er reagiert extrem empfindlich auf Luftströmungen.

### Strategie:
*   **Schutz:** Platziere ihn in einer Zone mit "ruhender Luft".
*   **Engineering-Trick:** Klebe ein Stück offenporigen Schaumstoff direkt über den Sensor. Das lässt den statischen Druck durch, filtert aber dynamische Windstöße (die beim Laufen entstehen) weg.
*   **Position:** Unkritisch in Bezug auf den CoM, solange er im Gehäuse geschützt ist.

---

## 4. ESP32 & AHT20: Hitze-Management
*   **ESP32:** Kann bei aktivem WLAN warm werden. Platziere ihn so, dass seine Abwärme nicht direkt den AHT20 oder die MPU6050 (Temperaturdrift!) aufheizt.
*   **AHT20:** Sollte am Rand des Gehäuses oder an einer Belüftungsöffnung sitzen, um die echte Umgebungstemperatur/Feuchtigkeit zu messen, nicht die "Gehäuse-Innentemperatur".

---

## 5. Zusammenfassung des Layouts (Draufsicht)

```text
       Vorn (Laufrichtung)
      +-----------------------+
      |       [AHT20]         |  <-- Außen (Klima)
      |                       |
      |     [ BMP280 ]        |  <-- Geschützt (Schaumstoff)
      |                       |
      |      [MPU6050]        |  <-- EXAKT IM CoM / MITTE
      |                       |
      |       [ESP32]         |  <-- Leicht versetzt
      |                       |
      |      [GY-271]         |  <-- Hinten/Oben (Magnet-Distanz)
      +-----------------------+
```

### 🛑 Profi-Tipp für Sim2Real:
Wenn du die IMU nicht exakt in den CoM bekommst, musst du den **Offset (X, Y, Z Distanz)** in deinem RL-Code in Isaac Sim hinterlegen (das sogenannte IMU-to-Body-Transform). Es ist aber 100x einfacher, sie mechanisch korrekt zu platzieren!
