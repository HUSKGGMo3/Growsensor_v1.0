# Growsensor – ESP32 Monitoring Node (Deutsch)

Growsensor ist eine leichte ESP32‑Firmware zur **Überwachung** von Grow‑Sensoren mit Web‑UI. Sie liest Temperatur, Luftfeuchte, CO₂, Lux/PPFD und Blatt‑Temperatur, berechnet VPD und visualisiert alles im Browser. **Keine Aktorsteuerung.**

## Was dieses Projekt ist / nicht ist
- ✅ Reines Monitoring: Sensorwerte, VPD/PPFD‑Berechnung, Dashboards, Logs.
- ❌ Keine Aktorsteuerung: keine Relais, Pumpen oder Automatisierung.

## Features
- Sensor‑Monitoring: Temp, Luftfeuchte, CO₂, Lux → PPFD, Blatt‑Temp.
- VPD‑Berechnung mit Wachstumsphasen und Zielbereich‑Status.
- Web‑UI mit Captive Portal, Live‑Dashboard, History‑Charts und Logs.
- Zeitbasierte Charts mit SNTP‑Sync + Zeitzonen.
- Authentifizierung mit Pflicht‑Passwortwechsel beim ersten Login.

## Unterstützte Hardware
- ESP32 (Arduino‑Framework)
- Sensoren:
  - BH1750 (Lux)
  - SHT31/SHT30 (Temp/Feuchte)
  - MLX90614 (Blatt‑Temp)
  - MH‑Z19/MH‑Z14 (CO₂)
- Pin‑Konfiguration in `src/main.cpp` (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Installation
### 1) Tooling installieren
- **VS Code** + **PlatformIO**‑Extension (empfohlen) oder PlatformIO CLI installieren.

### 2) Build & Flash (PlatformIO)
1. Dieses Repository in VS Code öffnen.
2. ESP32 per **USB‑Datenkabel** verbinden.
3. Den richtigen seriellen Port in PlatformIO auswählen.
4. Build & Upload:
   ```sh
   pio run -t upload
   ```
5. Optionaler serieller Monitor (115200 Baud):
   ```sh
   pio device monitor -b 115200
   ```

### 3) Erster Start & Setup
1. Nach dem Flashen startet ein Setup‑WLAN (SSID steht im Serial‑Log).
2. Mit dem Setup‑WLAN verbinden und die Setup‑Seite im Browser öffnen.
3. Login: `Admin` / `admin` und **Passwort ändern**.
4. Wi‑Fi und Sensoren konfigurieren.

Wenn etwas nicht klappt: USB neu stecken, anderen Port/Kabel testen, erneut bauen/flashen.

## Projektstruktur (Kurzüberblick)
- Firmware‑Entry: `src/main.cpp`
- Web‑UI Assets: `src/` (werden vom ESP ausgeliefert)
- PlatformIO Config: `platformio.ini`

## Verwandte Projekte & Roadmap
- Eine separate Controller‑Firmware (Aktorsteuerung) entsteht in einem anderen Repo. Dieses Repo bleibt Monitoring‑only und kündigt den Controller an, sobald er veröffentlicht ist.

## Lizenz
MIT License. Siehe [`LICENSE`](LICENSE).

## Haftungsausschluss
Experimentelle Firmware. Monitoring‑only, keine Garantien. Siehe [`DISCLAIMER.md`](DISCLAIMER.md).

## Updates (neueste zuerst)
### v0.3.5 (Zoom Stabil + WLAN Fix + Pi-Bridge Boost)
- Chart‑Zoom bleibt im Datenbereich (inkl. Touch‑Pinch).
- WLAN‑Scan liefert stabile Antworten (Throttle, leere Liste ohne Fehl‑Popup).
- Pi‑Bridge sendet WLAN‑Credentials nach Handshake, Log‑API + GUI‑Log + Failover‑Safe‑Mode.

### v0.3.4 (Factor Fix + WiFi Scan + Pi-Bridge Preview)
- LED/VPD‑Phasenlogik konsolidiert; PPFD/VPD‑Targets bleiben überall konsistent.
- WLAN‑Scan Dropdown stabilisiert (gedrosselte Async‑Scans + saubere JSON‑Responses).
- Chart‑Farbvorschau‑Label übernimmt die Linienfarbe.
- Neuer **Addons**‑Tab mit Raspberry‑Pi‑Growcontroller (Serial‑Bridge) Preview‑Toggle + Status.

### v0.3.3 (Stability Restoration)
- Sensor‑System wiederhergestellt (BH1750, SHT, MH‑Z19/MH‑Z14, VPD‑Berechnung).
- Cloud‑Stabilisierung; Refactor‑Fixes; Reports repariert.

### v0.3.2a
- Fixes für ArduinoJson Null‑Handling und doppelte Default‑Args.
