# 📄 Roadmap: Kommerzialisierung des IMU-Sensor-Servers in Europa (EU-Compliance)

**Projekt:** Hexapod RL Sensor Stack
**Produkt-Kategorie:** Elektronische Baugruppe / IoT-Sensor
**Zielmarkt:** Europa (EU/EWR)

Wenn wir dieses Low-Budget System als kommerzielles Produkt in Europa vertreiben wollen, müssen wir von der "Hobby-Verdrahtung" zu einem gesetzeskonformen Produkt übergehen. Hier sind die notwendigen Schritte nach **First Principles** der EU-Regulierung:

---

## 1. Gesetzliche Konformität (Zertifizierungen)

Ohne diese Siegel darf das Produkt in der EU nicht in Verkehr gebracht werden.

### 1.1 CE-Kennzeichnung (Conformité Européenne)
Das CE-Zeichen ist kein Qualitätssiegel, sondern eine Selbsterklärung, dass alle EU-Richtlinien eingehalten werden. Für unseren Sensor relevant:
*   **RED (Radio Equipment Directive 2014/53/EU):** Da der ESP32 WLAN/Bluetooth nutzt. Dies ist der teuerste Teil (EMV-Messungen in einem Labor).
*   **EMV-Richtlinie (2014/30/EU):** Nachweis, dass der Sensor andere Geräte nicht stört und selbst unempfindlich gegen Einstrahlung ist (kritisch bei Robotern mit Motoren).
*   **Niederspannungsrichtlinie (2014/35/EU):** Bei 5V Betrieb meist unkritisch, aber formal zu prüfen.

### 1.2 RoHS & REACH
*   **RoHS (Restriction of Hazardous Substances):** Nachweis, dass keine gefährlichen Stoffe (Blei, Quecksilber, Cadmium) im Lot oder in den Bauteilen enthalten sind.
*   **REACH:** Chemikalienverordnung der EU.

### 1.3 WEEE (Elektrogesetz)
*   Registrierung bei der **stiftung ear** (in Deutschland).
*   Das Symbol der "durchgestrichenen Mülltonne" muss auf das Produkt.
*   Zahlung von Entsorgungsgebühren pro verkauftem Gerät.

---

## 2. Hardware-Redesign (Vom Breadboard zum Produkt)

Um die Zertifizierung zu bestehen, muss die Hardware professionalisiert werden:

*   **Custom PCB:** Weg von den Jumper-Kabeln. Eine 2- oder 4-Layer Platine mit sauberen Ground-Planes ist zwingend für die EMV-Zertifizierung.
*   **ESD-Schutz:** Hinzufügen von TVS-Dioden an den Ein-/Ausgängen (SDA/SCL/Power), um Zerstörung durch statische Aufladung beim Anfassen zu verhindern.
*   **Gehäuse:** Ein IP-geschütztes Gehäuse (mind. IP30) aus flammhemmendem Material (UL94-V0 zertifiziert).

---

## 3. Produkthaftung & Rechtliches

*   **Bedienungsanleitung:** Muss zwingend in der Landessprache des Käufers vorliegen (inkl. Sicherheitshinweisen).
*   **Garantie & Gewährleistung:** In der EU sind 2 Jahre gesetzliche Gewährleistung für Endverbraucher Pflicht.
*   **Haftpflichtversicherung:** Eine Produkthaftpflichtversicherung ist für Robotik-Komponenten (Brandgefahr durch Fehlfunktion) absolut notwendig.

---

## 4. Kostenkalkulation (Schätzung für Kleinserie)

| Posten | Einmalige Kosten | Pro Gerät (bei 500 Stk.) |
| :--- | :--- | :--- |
| **Labor-Zertifizierung (RED/EMV)** | ca. 5.000€ - 8.000€ | ~12.00€ |
| **WEEE Registrierung** | ca. 500€ (p.a.) | ~1.00€ |
| **PCB & Bestückung (PCBA)** | - | ~15.00€ |
| **Gehäuse & Verpackung** | - | ~5.00€ |
| **Marketing & Logistik** | - | ~10.00€ |
| **GESAMT-HERSTELLKOSTEN** | - | **~43.00€** |

*Verkaufspreis-Ziel:* Um profitabel zu sein und Händlermargen zu bedienen, müsste das Produkt für ca. **89€ - 119€ (inkl. MwSt.)** angeboten werden.

---

## 5. First Principles Check: Macht es Sinn?

*   **Der USP:** Ein 500Hz Sensor-Stack, der direkt RL-ready Daten liefert.
*   **Konkurrenz:** Industrielle IMUs (Xsens, Bosch BMI) kosten oft 300€+. Billige China-Module haben keine CE-Zertifizierung für den gewerblichen Einsatz.
*   **Fazit:** Wenn wir die Firmware als "Open Source" belassen und nur die zertifizierte Hardware verkaufen, minimieren wir das Software-Haftungsrisiko.

---

**Nächster Schritt:** Sollen wir ein **Lastenheft** für die erste Prototypen-Platine (PCB) erstellen, die alle EMV-Schutzmaßnahmen enthält? [[reply_to_current]]
