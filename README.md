# Growsensor – ESP32 Monitoring Node

**Current release: v0.2.6 (untested / community preview)**

### Hotfix v0.2.6
- Adds NTP-backed real-time clocks with timezone selection in the header and a visible “Zeit nicht synchron” badge until sync succeeds.
- History/telemetry now return epoch millisecond timestamps so all charts render wall-clock time (including hover + detail views) with local-time axis labels; hover overlays now show full axes with start/mid/end labels instead of cropped canvases.
- Persistent timezone preference (NVS) with SNTP retries (pool.ntp.org, time.nist.gov, time.google.com) and graceful fallback to relative time until synced.
- New bucketed charting: Live/6h views aggregate into 5-minute buckets, 24h charts aggregate into 15-minute buckets (stored on-device to avoid RAM spikes). The dashboard’s large chart is now a 24h/15m view with a metric dropdown.
- Hotfix v0.2.6 (untested): Charts 6h/24h fixed, axes label collision avoidance, hover charts now full-tile.
- Hotfix v0.2.6 (untested): Time-axis tick strategy with more (auto-skipped) labels, padded Y-axes so single values are visible, full-height hover charts, and per-device color dropdowns (persisted in `localStorage`) with legends showing the device ID.
- Hotfix v0.2.6 (untested): MH-Z14 (CO₂) added to the sensor templates (UART, mapped to the MH-Z19 driver path).
- Hotfix v0.2.6 (untested): VPD heatmap fully re-renders when VPD factors/targets change so the green target zone and heatmap overlay stay in sync.

### Patch: Wi-Fi reconnect & stability (post v0.2.6)
- Wi-Fi change no longer requires Dev mode: `/api/wifi` saves credentials instantly and reconnects asynchronously with mDNS (`growsensor.local` by default). The UI shows a reconnect panel, polls `growsensor.local/api/ping` every ~1.5s for up to 60s, and offers a fallback hint to the setup AP if mDNS fails; `/api/status` now exposes SSID/IP/RSSI/hostname/connecting for the UI and recovery flows.
- Soft restart is now `/api/restart` (pure `ESP.restart()` with a “NO ERASE” log). Destructive operations are isolated in `/api/factory-reset` with `RESET` confirmation; preferences are split into wifi/sensors/system/ui namespaces to avoid accidental global clears.
- Sensor cooldowns: Lux/PPFD, climate, leaf, and CO₂ sensors are rate-limited to two reads per minute. Skipped polls no longer push history points, easing I²C/UART load while keeping health tracking intact.
- Charts/UX: mini-charts are always visible behind values with dimmed overlays; X-axis ticks scale with chart width to avoid label collisions; the “all metrics” mixed graph is removed. Metrics without real data stay hidden from selectors/legends, live charts require recent samples, and telemetry now carries `*_last`/`*_ever` timestamps to drop ghost series.
- Anti-lockout: failed Wi-Fi connects fall back to the setup AP after a short timeout, while mDNS starts automatically on every successful STA connection.

Lightweight ESP32 monitoring firmware with a WebUI for grow environments. It reads multiple sensors, estimates PPFD, computes VPD per growth stage, and surfaces everything in a browser UI with Wi‑Fi setup and partner/supporter info. It does **not** drive actuators.

## What this project is / is not
- Monitoring only: reads sensors, computes PPFD/VPD, shows dashboards and logs.
- No actuator control yet: no relays or automation are driven by this firmware.

## Features (as of v0.2.x)
- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages (seedling/veg/bloom/late bloom) and status (under / in / over target).
- Web-based UI with captive portal setup, live dashboard, 24h chart, averages, and logs.
- Time-aware charts with SNTP-based epoch timestamps; timezone-aware axes (Europe/Berlin, UTC, America/New_York, Asia/Tokyo, more) and a header clock badge that indicates sync state.
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
- Sensors: BH1750 (Lux), SHT31/SHT30 (Temp/Humidity), MLX90614 (Leaf Temp), MH-Z19/MH-Z14 series (CO₂).
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
- Dashboard tiles can now be collapsed or expanded via a small eye icon; collapsed tiles shrink to a header-only view (no values or hover charts) and clicks expand them without opening the detail modal. State is stored per tile in `localStorage` (`tile_visibility_v026`) and defaults to visible for all metrics.
- Hotfix: restored broken UI interactions (dashboard navigation, dev mode modal, hover/detail charts) and Wi‑Fi information (connected state, SSID/IP, RSSI bars, static-IP toggle).
- NTP sync (pool.ntp.org, time.nist.gov, time.google.com) after Wi‑Fi connect with periodic refresh and a persisted timezone (Preferences/NVS).
- `/api/telemetry` and `/api/history` return epoch millis; the UI renders local time axes (or relative mm:ss when unsynced) in main, hover, and detail charts. Hover mini-charts now draw full-size canvases with X/Y axes and time labels.
- Chart ranges are bucketed to save memory: Live/6h use 5-minute buckets, 24h uses 15-minute buckets. The detail modal gains a 24h tab, and the dashboard’s big chart now shows a selectable metric over the last 24h (15m resolution).
- Header timezone dropdown with persistent selection and live clock.
- Chart UX polish: more X-axis ticks with collision avoidance, padded Y-axes, per-metric color palette with device-ID legend in the large 24h chart and detail modal (colors are stored per device in `localStorage`).
- Sensor templates expanded with MH-Z14 (CO₂) mapped to the existing UART driver.
  (v0.2.6 remains untested – please use with care.)

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
