# Growsensor – ESP32 Monitoring Node

Growsensor is a lightweight ESP32 firmware that **monitors** grow-environment sensors and serves a Web UI. It reads temperature, humidity, CO₂, lux/PPFD, and leaf temperature, calculates VPD, and visualizes everything in the browser. **It does not control actuators.**

## What this project is / is not
- ✅ Monitoring only: sensor reads, VPD/PPFD calculations, dashboards, and logs.
- ❌ No actuator control: no relays, pumps, or automation logic.

## Features
- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages and target-band status.
- Web UI with captive portal setup, live dashboard, history charts, and logs.
- Time-based charts with SNTP time sync + timezone support.
- Authentication with mandatory password change on first login.

## Supported Hardware
- ESP32 (Arduino framework)
- Sensors:
  - BH1750 (Lux)
  - SHT31/SHT30 (Temp/Humidity)
  - MLX90614 (Leaf Temp)
  - MH-Z19/MH-Z14 (CO₂)
- Pin configuration lives in `src/main.cpp` (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Installation
### 1) Install tooling
- Install **VS Code** and the **PlatformIO** extension (recommended), or PlatformIO CLI.

### 2) Build & flash (PlatformIO)
1. Open this repository in VS Code.
2. Connect the ESP32 via **USB data cable**.
3. Select the correct serial port in PlatformIO.
4. Build and upload:
   ```sh
   pio run -t upload
   ```
5. Optional serial monitor (115200 baud):
   ```sh
   pio device monitor -b 115200
   ```

### 3) First boot & setup
1. After flashing, the device starts a setup Wi‑Fi (see serial log for SSID).
2. Connect to the setup Wi‑Fi and open the setup page in your browser.
3. Log in with `Admin` / `admin` and **change the password**.
4. Configure Wi‑Fi and enabled sensors.

If anything fails: replug the USB cable, try another port, and rebuild/upload.

## Project structure (quick map)
- Firmware entry point: `src/main.cpp`
- Web UI assets: `src/` (served from firmware)
- PlatformIO config: `platformio.ini`

## Related projects & roadmap
- A separate controller firmware is being prepared in another repo (actuator control). This repository stays monitoring-only and will announce the controller once it is published.

## License
This project is licensed under the **MIT License**. See [`LICENSE`](LICENSE).

## Disclaimer
Experimental firmware. Monitoring only, no warranties. See [`DISCLAIMER.md`](DISCLAIMER.md).

## Updates (latest at top)
### v0.3.5 (Zoom Stabil + WLAN Fix + Pi-Bridge Boost)
- Chart-zoom stays in bounds (including touch pinch).
- Wi‑Fi scan is throttled and returns stable responses (no error popup on empty results).
- Pi-Bridge sends Wi‑Fi credentials after handshake, includes log API + GUI log + failover safe mode.

### v0.3.4 (Factor Fix + WiFi Scan + Pi-Bridge Preview)
- LED/VPD stage logic consolidated; PPFD/VPD targets update consistently across UI.
- Wi‑Fi scan dropdown stabilized with throttled async scans and clean JSON.
- Chart color preview label mirrors the selected line color.
- New **Addons** tab with Raspberry Pi Growcontroller (Serial Bridge) preview toggle and status.

### v0.3.3 (Stability Restoration)
- Sensor system restored (BH1750, SHT, MH‑Z19/MH‑Z14, VPD calculations).
- Cloud stabilization; refactor fixes; reports repaired.

### v0.3.2a
- Fixes for ArduinoJson null handling and duplicate default args.
