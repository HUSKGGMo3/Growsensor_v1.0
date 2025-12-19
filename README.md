# Growcontroller v0.1.2 (ESP32)

Firmware für das GrowSensor-Gehäuse (ESP32) mit Captive Portal, geschützter Weboberfläche, Sensor-Manager, VPD-Auswertung pro Pflanzenstadium und 24h-Verlaufsdiagrammen.

## Hauptfeatures
- **Login-Pflicht**: Standard-Creds `Admin` / `admin`, beim ersten Login muss das Passwort geändert werden (danach WLAN-Konfiguration möglich).
- **Wi-Fi-Wizard**: WLAN-Scan mit Dropdown, optional Static IP, Erfolgsmeldung („Verbunden mit deiner Pflanze“) bzw. Fehler („Falsches Passwort“).
- **Sensor-Manager**: Sensoren (BH1750, SHT31/SHT41, MLX90614, MH-Z14) ein-/ausschalten, Gesundheitsstatus (aktiv/keine Daten/abgeschaltet bei Stalls >4h).
- **VPD pro Stadium**: Steckling/Sämling, Vegitativ, Blütephase, Späteblüte – Zielkorridor (kPa) wird angezeigt, Berechnung wird stadienabhängig skaliert.
- **Live-Dashboard**: Lux → PPFD, CO₂, Umgebungstemp, Feuchte, Leaf-Temp, VPD sowie Ø-Werte der letzten 24h; 24h-Chart (Lux/CO₂/Temp/VPD, 5-min-Raster).
- **Logs**: Letzte ~6h im UI ansehen oder herunterladen.
- **Stall-Watchdog**: Bleiben Messwerte >4h unverändert, wird der Sensor deaktiviert und im Sensor-Tab als fehlerhaft markiert.
- **Persistenz**: WLAN, Static IP, Spektrum, VPD-Stadium, Sensor-Schalter, Login-Daten in NVS.

## Hardware-Pins
| Sensor | Adresse/Interface | ESP32 Pins |
| --- | --- | --- |
| BH1750 | I²C `0x23` | SDA `21`, SCL `22` |
| SHT31/SHT41 | I²C `0x44` | SDA `21`, SCL `22` |
| MLX90614 | I²C `0x5A` | SDA `21`, SCL `22` |
| MH-Z14 | UART 9600 | RX `16` (ESP32), TX `17` (ESP32) |

> Pins können in `src/main.cpp` angepasst werden (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Build & Flash (PlatformIO)
1. PlatformIO CLI oder VS Code Extension installieren.
2. ESP32 per USB verbinden.
3. Kompilieren & flashen:
   ```sh
   pio run -t upload
   ```
4. Serielles Log (115200 Baud):
   ```sh
   pio device monitor -b 115200
   ```

## Erstinbetriebnahme
1. Gerät starten → AP `GrowSensor-Setup` (Passwort `growcontrol`).
2. Browser öffnet Captive Portal (`http://192.168.4.1`).
3. **Login:** `Admin` / `admin`. Direkt neues Passwort setzen (Pflicht).
4. **Wi-Fi-Wizard:** „Netzwerke suchen“ → SSID wählen → Passwort (optional Static IP) → „Verbinden & Speichern“.
5. Bei Erfolg grüne Meldung „Verbunden mit deiner Pflanze“. Bei falschem Passwort rote Meldung.

## Weboberfläche
- **Dashboard:** Live-Werte + Ø 24h, VPD-Ziel je Stadium, 24h-Chart (Lux/CO₂/Temp/VPD).
- **Spektrum & VPD-Stadium:** Spektrum (Lux→PPFD-Faktor) und Pflanzenstadium wählen.
- **Sensoren:** Aktiv/Deaktivieren, Status „aktiv“ / „keine Daten“ / „failed (stalled)“.
- **Logs:** Letzte ~6h ansehen und herunterladen.

## API (auth-pflichtig, Header `X-Auth` aus Login)
- `POST /api/auth` (`user`, `pass`) → `{ token, mustChange }`
- `POST /api/auth/change` (`new_pass`, optional `new_user`)
- `GET/POST /api/settings` (`channel`, `vpd_stage`, Static-IP-Felder)
- `POST /api/wifi` (`ssid`, `pass`, `static=0|1`, `ip`, `gw`, `sn`)
- `GET /api/telemetry` → `{ lux, ppfd, co2, temp, humidity, leaf, vpd, vpd_low, vpd_high }`
- `GET|POST /api/sensors`
- `GET /api/networks`
- `POST /api/reset`
- `GET /api/logs`

## Hinweise
- VPD-Zielkorridore (kPa):
  - Steckling/Sämling: 0.4–0.8
  - Vegitativ: 0.8–1.2
  - Blütephase: 1.0–1.4
  - Späteblüte: 0.8–1.2
- Watchdog deaktiviert Sensoren nach >4h unverändertem Messwert; Status wird im Sensor-Tab angezeigt.
- MH-Z14 Auto-Kalibrierung bleibt deaktiviert (`autoCalibration(false)`).

## Trouble Shooting
- Keine Werte? Sensor-Tab prüfen (Status „keine Daten“/„failed“) und Verkabelung kontrollieren.
- WLAN-Probleme? Im AP-Modus erneut verbinden, einloggen und WLAN neu speichern.
