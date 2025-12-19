# Growsensor – ESP32 Monitoring Node

**Current release: v0.2.0 (untested / community preview)**

Growsensor is an all-in-one ESP32 monitoring node for indoor grows. It solves basic telemetry and visibility: reads multiple sensors, estimates PPFD, computes VPD per growth stage, and shows everything in a web UI with Wi-Fi setup and simple partner/supporter info. It does **not** drive any actuators.

Growsensor (monitoring) vs. future Growcontroller (control): this project only collects and visualizes data. A separate future Growcontroller project will handle automation and device control once ready.

# ESP32 Grow Sensor / Monitoring Node

**Current release: v0.2.0 (untested / community preview)**

Lightweight ESP32 monitoring firmware with a WebUI for grow environments. Provides sensor telemetry, PPFD/VPD insights, Wi-Fi setup, and a partner/supporter module. No actuator control is implemented yet.



## What this project is / is not
- Monitoring only: reads sensors, computes PPFD/VPD, shows dashboards and logs.
- No actuator control yet: no relays or automation are driven by this firmware.



# ESP32 Grow Sensor / Monitoring Node

**Current release: v0.2.0 (untested / community preview)**

Lightweight ESP32 monitoring firmware with a WebUI for grow environments. Provides sensor telemetry, PPFD/VPD insights, Wi-Fi setup, and a partner/supporter module. No actuator control is implemented yet.


## What this project is / is not
- Monitoring only: reads sensors, computes PPFD/VPD, shows dashboards and logs.
- No actuator control yet: no relays or automation are driven by this firmware.


- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages (seedling/veg/bloom/late bloom) and status (under / in / over target).
- Web-based UI with captive portal setup, live dashboard, 24h chart, averages, and logs.
- Authentication with forced password change on first login.
- Partner / Supporter module stored locally and shown in the UI.

## Planned / Future
- Growcontroller (automation and device control)
- Stabilization, long-term testing, and production readiness
- Additional sensor/driver coverage and usability improvements

**Current release: v0.2.0 (untested / community preview)**

Lightweight ESP32 monitoring firmware with a WebUI for grow environments. Provides sensor telemetry, PPFD/VPD insights, Wi-Fi setup, and a partner/supporter module. No actuator control is implemented yet.

## What this project is / is not
- Monitoring only: reads sensors, computes PPFD/VPD, shows dashboards and logs.
- No actuator control yet: no relays or automation are driven by this firmware.


## Features (as of v0.2.0)
- Sensor monitoring: Temp, Humidity, CO₂, Lux → PPFD, Leaf Temp.
- VPD calculation with growth stages (seedling/veg/bloom/late bloom) and status (under / in / over target).
- Web-based UI with captive portal setup, live dashboard, 24h chart, averages, and logs.
- Authentication with forced password change on first login.
- Partner / Supporter module stored locally and shown in the UI.



## Supported Hardware
- ESP32 (classic, Arduino framework)
- Sensors: BH1750 (Lux), SHT31/SHT30 (Temp/Humidity), MLX90614 (Leaf Temp), MH-Z19 series (CO₂).
- I²C pins and CO₂ UART pins are configurable in `src/main.cpp` (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Security & Login
- Default login: `Admin` / `admin`
- Password change is required on first login before Wi-Fi changes are allowed.
- A master password exists for recovery; it is not exposed here—set your own at build time if needed.

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

## Stability notice
- v0.2.0 is untested and provided as a community preview. Use at your own risk.

## ESPHome option
- Du kannst den ESP32 auch mit ESPHome flashen, um die Sensoren direkt in ESPHome zu nutzen. Lies dazu die ESPHome-Dokumentation und hinterlege die Sensoren korrekt in deiner ESPHome-Konfiguration.

## License
- Non-commercial open source license (see `LICENSE`). You may view, use, and modify the code and contribute via pull requests.
- Commercial use (including selling devices or services, or paid products) requires explicit permission.
- Contributions are welcome; modified versions must keep the same license and attribution.
- Deutsche Version: siehe `README.de.md` für eine vollständige Zusammenfassung auf Deutsch.

## Contributing
- Contributions via pull requests are welcome (see `CONTRIBUTING.md`). Please respect the non-commercial license.
- Experimental hardware/software: see `DISCLAIMER.md` before deploying.

## Contributing
- Contributions via pull requests are welcome (see `CONTRIBUTING.md`). Please respect the non-commercial license.
- Experimental hardware/software: see `DISCLAIMER.md` before deploying.

## License
- Non-commercial open source license (see `LICENSE`). You may view, use, and modify the code and contribute via pull requests.
- Commercial use (including selling devices or services, or paid products) requires explicit permission.
- Contributions are welcome; modified versions must keep the same license and attribution.