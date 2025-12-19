# Growsensor – ESP32 Monitoring Node

**Current release: v0.2.6 (untested / community preview)**

Lightweight ESP32 monitoring firmware with a WebUI for grow environments. It reads multiple sensors, estimates PPFD, computes VPD per growth stage, and surfaces everything in a browser UI with Wi‑Fi setup and partner/supporter info. It does **not** drive actuators.

## What this project is / is not
- Monitoring only: reads sensors, computes PPFD/VPD, shows dashboards and logs.
- No actuator control yet: no relays or automation are driven by this firmware.

## Features (as of v0.2.x)
- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages (seedling/veg/bloom/late bloom) and status (under / in / over target).
- Web-based UI with captive portal setup, live dashboard, 24h chart, averages, and logs.
- Authentication with forced password change on first login.
- Partner / Supporter module stored locally and shown in the UI.

## UI/UX refresh (v0.2.x dashboard update)
- Two in-app views: **Dashboard** (default) and **Sensoren** (sensor management) with client-side switching.
- Metric tiles are fully clickable; a detail modal opens with live and 6h charts for Lux, PPFD, CO₂, Temp, Humidity, Leaf, and VPD, with click-logging to debug overlay blockers.
- Sensor tiles show per-metric status LEDs (green = valid data, yellow = stale/invalid, gray = disabled/not present) driven by telemetry flags.
- VPD tile uses a full-background gradient “chart look” with target band highlight and value marker (0.0–2.0 kPa scale, marker hidden when no valid data).
- Dedicated **Sensoren** page with active/available sensor tiles and a “+ Sensor hinzufügen” wizard (category/type selection, default pins locked until advanced override, restart hint).
- Wi-Fi card collapses to a pulsing “connected” status with IP info; an advanced toggle reveals the setup form and static-IP fields only when enabled.
- Telemetry JSON exposes sensor presence/enabled/ok/age fields plus Wi-Fi details (`ip`, `gw`, `sn`, `ap_mode`) for UI state.

## Supported Hardware
- ESP32 (classic, Arduino framework)
- Sensors: BH1750 (Lux), SHT31/SHT30 (Temp/Humidity), MLX90614 (Leaf Temp), MH-Z19 series (CO₂).
- I²C pins and CO₂ UART pins are configurable in `src/main.cpp` (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Security & Login
- Default login: `Admin` / `admin`
- Password change is required on first login before Wi-Fi changes are allowed.
- A master password exists for recovery; set your own at build time if needed.

## Build & Flash (PlatformIO)
1. Install PlatformIO CLI or VS Code + PlatformIO extension.
2. Connect the ESP32 via USB.
3. Build and flash:
   ```sh
   pio run -t upload
   ```
4. Serial monitor (115200 baud):
   ```sh
   pio device monitor -b 115200
   ```

## v0.2.6 Changes
- 6h history is always downsampled to 1-minute buckets for every metric with consistent `live` / `6h` ranges.
- Mini- and detail charts show proper time/value axes with the right units and now reliably render `/api/history` data without empty graphs.
- History JSON trims decimals per metric (Lux/Temp/Humidity 1, CO₂ 0, VPD 3, PPFD 1) to keep payloads and RAM usage lean; log output prunes when heap is low and caps API responses.
- Sensor wizard modal now has a blurred overlay, pop-in animation, ESC/overlay-close debounce to avoid accidental closes, and defensive checks to prevent UI breakage.

## v0.2.5 Changes
- VPD korrekt: Targets statt Skalierung + Apply Button für Stages.
- VPD Heatmap klein/groß konsistent mit rotem Live-Punkt.
- Popup Graphs mit Live/6h per `/api/history` und Achsen/Labels.
- Sensor Add: gefiltert nach Kategorie + Advanced Pin Config Warnung.
- WiFi Panel: RSSI Balken, IP Ansicht und stabiler „Wi-Fi ändern“-Toggle.
- Header Hover Particle Animation („grün sprühen“).

## Changelog v0.2.2
- Fix: Charts in der Popup-Ansicht erhalten wieder Live- und 6h-Daten für alle Sensoren.
- Neu: VPD-Heatmap-Chart im Stil gängiger VPD-Tabellen (kleine Kachel + großes Modal).
- Verbesserung: Popup-Chart-UX mit klarerer Skalierung und optionalem Dev-Debug.

## Stability notice
- v0.2.6 is untested and provided as a community preview. Use at your own risk.

## ESPHome option
- You can also flash the ESP32 with ESPHome to use the sensors directly in ESPHome. See ESPHome documentation and configure the sensors accordingly.

## License
- Non-commercial open source license (see `LICENSE`). You may view, use, and modify the code and contribute via pull requests.
- Commercial use (including selling devices or services, or paid products) requires explicit permission.
- Contributions are welcome; modified versions must keep the same license and attribution.
- German version: see `README.de.md` for a full summary in German.

## Contributing
- Contributions via pull requests are welcome (see `CONTRIBUTING.md`). Please respect the non-commercial license.
- Experimental hardware/software: see `DISCLAIMER.md` before deploying.
