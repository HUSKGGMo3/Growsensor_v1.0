# Growsensor – ESP32 Monitoring Node

**Current releases:**

- **GrowSensor – v1.1.0-s3 (ESP32-S3, UM ProS3 16MB + 8MB PSRAM)** – default build in the PlatformIO `src/` folder using `env:esp32s3_16r8` (Arduino, USB-CDC on boot, PSRAM enabled).
- **GrowSensor – v1.0.1-pro16m (ESP32 Pro 16MB)** – archived in `/legacy_esp32` for older PSRAM DevKit-style boards.
- **GrowSensor – v0.3.3 (Classic ESP32)** – archived in `/legacy_esp32` for standard ESP32-DevKit boards.

**Project layout:**

- `/src` – actively maintained ESP32-S3 firmware (UM ProS3, 16MB flash + 8MB PSRAM).
- `/legacy_esp32` – snapshot of the previous ESP32/ESP32-Pro firmware and PlatformIO configs.

## Beginner-Friendly Install (for Dummies)
You only need a USB cable, an ESP32-S3 (UM ProS3), and a laptop/PC.

1. Install **VS Code** and the **PlatformIO** extension (search “PlatformIO IDE” in VS Code extensions).
2. Open this repository folder in VS Code.
3. Connect the ESP32-S3 via USB (use a data cable, not charge-only).
4. In PlatformIO, select the correct **Serial Port** for your ESP32-S3.
5. Build and flash the S3 target: `pio run -e esp32s3_16r8 -t upload`
6. Open the **Serial Monitor** at **115200 baud** to see logs.
7. After reboot, connect your phone/PC to the device’s setup Wi‑Fi (shown in the serial log), then open the setup page in your browser.
8. Log in with `Admin` / `admin`, change the password, and configure Wi‑Fi/sensors.

Legacy boards (ESP32 classic / ESP32 Pro 16MB) remain available under `legacy_esp32/`:
`pio run -d legacy_esp32 -e esp32pro16m_legacy` or `pio run -d legacy_esp32 -e esp32classic_legacy`.

If anything fails: unplug/replug, try another USB cable/port, and rebuild/upload again.

### Hotfix v1.0.1-pro16m (partition stability + auto-detect)
- Added `partitions.csv` (previously `partitions_16MB.csv`) to the project root with dual OTA slots + SPIFFS for 16MB ESP32 Pro boards; `board_upload.flash_size = 16MB` is enforced.
- Build hook auto-generates safe OTA partition maps for 8MB/4MB devices when the configured flash size is smaller, preventing CSV lookup errors and overlaps.
- Firmware/version string bumped to `v1.0.1-pro16m` for the Pro target.

### Release v0.4.0 (ESP32 Pro, 16MB, PSRAM)
- New PlatformIO target **esp32pro16m** (default) for 16MB ESP32 Pro boards with external antenna & PSRAM.
- Classic build kept as **esp32classic** (unchanged feature set) for DevKit / legacy boards.
- Firmware strings clarify channel/board (v0.3.3 Classic vs. v0.4.0 Pro) in APIs and cloud payloads.
- Partition map for Pro build supports dual OTA slots + large SPIFFS for cached assets/logs.

### Classic Patch v0.3.5 (Zoom Stabil + WLAN Fix + Pi-Bridge Boost)

- Chart-Zoom in der Detailansicht hält das Datenfenster im sichtbaren Bereich (inkl. Touch-Pinch).
- WLAN-Scan liefert stabile Antworten, inkl. leerer Ergebnisse ohne Fehler-Popup + Throttle.
- Pi-Bridge sendet WLAN-Credentials nach Handshake, Log-API + GUI-Log + Failover-Safe-Mode.

### Classic Patch v0.3.4 (Factor Fix + WiFi Scan + Pi-Bridge Preview)
- LED/VPD phase logic consolidated: VPD target + PPFD scale now update consistently across tiles, header, and heatmaps.
- Wi‑Fi scan dropdown fixed with throttled async scans and stable JSON responses.
- Chart color preview label now mirrors the selected line color.
- New **Addons** tab with a Raspberry Pi Growcontroller (Serial Bridge) preview toggle and status.

### Release v0.4.0 (Documentation + Install Guide Refresh)
- Firmware version bumped to v0.4.0.
- Beginner-friendly installation guide added (English + German docs).

### Patch v0.3.5 (Stability + Insight Update)
- Adds periodic cloud debug snapshots every 5 minutes (`/GrowSensor/<deviceId>/logs/debug_YYYY-MM-DD.txt`) with Wi‑Fi, cloud queue/backoff, sensor health, and loop diagnostics; no local persistence beyond the short upload cache.
- VPD header KPI now color-codes against the stage target (green within ±10%, yellow outside, red critical) with a subtle pulse that matches existing UI styling.
- Hover charts are preloaded (throttled) so mini graphs render immediately without hover delays, while keeping 6h/24h/long-range logic intact.
- Factor changes (PPFD channel + VPD stage) now trigger safe recomputation and redraws without UI crashes, guarding against NaNs in VPD heatmaps and charts.

### Patch v0.3.3 (Stability Restoration)
- Sensor system restored: BH1750, climate (SHT), CO₂ (MH-Z19/MH-Z14), VPD calculation, and telemetry status fields behave like pre-refactor.
- Cloud stabilized: archive stays available but no longer blocks WebUI or range requests; failures populate `cloudStatus.lastError` without spamming.
- Refactor fixes: core helpers restored and API ranges no longer depend on cloud HEAD probes.
- Reports repaired: `dayKeysBetween()` restored to deterministic date ranges; report generation uses the repaired day list.
- Acceptance checklist: firmware builds, WebUI stays responsive, live tiles + charts update, cloud upload stable, no reset loops or timeout spam.

### Patch v0.3.2 (experimental, Cloud Primary mode)
- Storage modes (`LOCAL_ONLY`, `CLOUD_PRIMARY`, `LOCAL_FALLBACK`) gate persistence: when the cloud is online the ESP keeps only RAM ring buffers (live/6h/24h) and blocks local history writes, while 15-minute checkpoints upload the active day and daily rollovers queue JSON for the cloud.
- WebDAV uploads target `/GrowSensor/<deviceId>/daily/YYYY-MM/YYYY-MM-DD.json` (HTTP basic auth, LAN-only) with date/timezone/firmware metadata, per-metric aggregates (avg/min/max/samples/last), and optional hourly bins; folders are auto-created via MKCOL and uploads are retried with a bounded, upserting queue.
- Cloud health pings every ~30s to define connectivity; the UI shows status/last sync, hides 1M/3M/4M ranges unless `cloudEnabled && cloudConnected`, surfaces an offline notice + retry button, and long-range charts pull daily JSONs directly from the cloud while offline falling back to the 24h local buffer.
- Hardened cloud state & runtime flags: `/api/cloud` now distinguishes persisted vs. runtime enablement, recording, queue size, last upload/test path/time, reasons, and failure counters. The worker ticks ~1.5s with tiered retry backoff (5s/15s/60s, max 5 attempts), enqueues a recording-start event immediately, and pushes per-minute recording snapshots under `/GrowSensor/<deviceId>/samples/...`.
- New `/api/cloud/test` endpoint and “Sende Test” UI button create `TestCloud_<deviceId>_<timestamp>.txt` via MKCOL/PUT under `/GrowSensor/<deviceId>/` (LAN WebDAV), returning HTTP code/path/bytes for end-to-end verification.
- Nextcloud WebDAV uses plain HTTP in trusted LANs; upload root remains `/GrowSensor/<deviceId>/` and the UI keeps the config persisted across reboots.
- Hotfix v0.3.2: fixed missing declarations/helpers in the cloud recording pipeline; restored successful build.
- Hotfix v0.3.2a: fixed ArduinoJson null assignments and removed duplicate default arguments to restore builds.

### Patch v0.3.1 (experimental, HTTP-only LAN cloud)
- New **Cloud** tab (visible for all users) to configure LAN-only Nextcloud WebDAV logging over HTTP. Stores credentials in a dedicated `cloud` namespace, never wipes Wi‑Fi/settings, and warns when using HTTP (plaintext). Includes Test/Start/Stop controls, queue/failure counters, and a retention selector (1–4 months).
- Daily aggregates per metric (avg/min/max/last/count) are kept locally (ring buffer, persisted) and uploaded once per day as JSON to `/GrowSensor/<device>/daily/YYYY/MM/DD.json`. Upload queue is bounded and retried with backoff; directories are created via idempotent MKCOL. UI long-range charts (1M/3M/4M) read local aggregates, appear only when cloud is enabled/connected, and keep the app offline-capable.
- Header KPI bar centered; trend arrows sit inline with values and use hardened trend detection with thresholds/hysteresis. Temp/Humidity/CO₂ show green (strong) / yellow (soft) trends; VPD arrows show direction while color indicates “toward target” (green), “away” (red), or neutral.
- Sensor reads stay rate-limited to max 2/min (cooldowns already enforced), and version banner bumped to v0.3.1.

### Patch v0.3 (untested)
- Chart hover now snaps to the nearest timestamp with a series-colored tooltip, vertical cursor line, and marker dot on all charts (detail modal, 24h main chart, and mini hover charts). Tooltips stay next to the cursor and use client-side data only.
- Sticky header KPI bar shows Temp / Humidity / CO₂ / VPD with trend arrows (debounced thresholds) and VPD target proximity coloring (toward target = green, away = red).
- Tile status indicators now pulse based on state (fast for healthy green, slower for yellow/stale, idle for offline); respects `prefers-reduced-motion`.
- Version banner bumped to v0.3 (untested).

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

## Features (as of v1.0)
- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages (seedling/veg/bloom/late bloom) and status (under / in / over target).
- Web-based UI with captive portal setup, live dashboard, 24h chart, averages, and logs.
- Time-aware charts with SNTP-based epoch timestamps; timezone-aware axes (Europe/Berlin, UTC, America/New_York, Asia/Tokyo, more) and a header clock badge that indicates sync state.
- Until the clock syncs, telemetry/cloud uploads keep monotonic timestamps and daily aggregation is labeled as `unsynced` to avoid fake day boundaries.
- Authentication with forced password change on first login.
- Partner / Supporter module stored locally and shown in the UI.

## UI/UX refresh (v1.0 dashboard update)
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
2. Connect the ESP32-S3 (UM ProS3) via USB.
3. Build and flash the S3 firmware (`/src`) with esptool (bootloader + partitions + boot_app0 + app):
   ```sh
   pio run -e esp32s3_16r8 -t upload
   ```
   For a forced full rewrite of every image (handy after a bad flash), use:
   ```sh
   pio run -e esp32s3_16r8 -t uploadfull
   ```
4. Serial monitor (115200 baud):
   ```sh
   pio device monitor -b 115200
   ```
5. The S3 build pins the 16MB partition table at `partitions.csv` (labelled `nvs` + `otadata` + dual OTA app slots + SPIFFS for the remaining space). The guard script validates offsets and auto-writes a matching CSV when the flash size changes, preventing missing/overlapping partition errors and keeping `Preferences.begin()` aligned with the `nvs` partition on UM ProS3 (16MB flash, 8MB PSRAM). Core dumps to flash are disabled to avoid warnings when no dedicated coredump slot exists.
   If you ever see `Preferences.begin(): nvs_open failed: NOT_FOUND`, the device was flashed with a mismatched/absent NVS partition—re-uploading with the commands above realigns the offsets, and the firmware now probes/repairs NVS at boot (erase + re-init on version/space errors) before touching Preferences while falling back to RAM-only settings plus a single warning if storage stays unavailable.
6. For classic ESP32 boards, use the archived project under `legacy_esp32/`:
   ```sh
   pio run -d legacy_esp32 -e esp32pro16m_legacy -t upload
   # or
   pio run -d legacy_esp32 -e esp32classic_legacy -t upload
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

## v0.3.2a Changes
- Fix: ArduinoJson null handling in recording payload; removed duplicate default args to restore compilation.

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
