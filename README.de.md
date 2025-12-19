# ESP32 Grow Sensor / Monitoring Node (Deutsch)

**Aktuelle Version: v0.2.0 (ungetestet / Community-Preview)**

Leichtgewichtige, reine Monitoring-Firmware für einen ESP32 mit WebUI. Bietet Sensordaten, PPFD/VPD-Auswertung, Wi-Fi-Setup und ein Partner/Supporter-Modul. Keine Aktorsteuerung vorhanden.

## Was dieses Projekt ist / nicht ist
- Nur Monitoring: liest Sensoren aus, berechnet PPFD/VPD, zeigt Dashboards und Logs.
- Keine Aktorsteuerung: Relais/Automatisierung sind nicht implementiert.

## Features (Stand v0.2.0)
- Sensor-Monitoring: Temperatur, Luftfeuchte, CO₂, Lux → PPFD, Blatt-Temperatur.
- VPD-Berechnung mit Wachstumsphasen (Steckling/Veg/Blüte/Späte Blüte) und Status (unter / im / über Ziel).
- Webbasierte UI mit Captive Portal, Live-Dashboard, 24h-Chart, Mittelwerten und Logs.
+- Authentifizierung mit Pflicht-Passwortwechsel beim ersten Login.
- Partner-/Supporter-Modul (lokal gespeichert, in der UI sichtbar).

## Unterstützte Hardware
- ESP32 (klassisch, Arduino-Framework)
- Sensoren: BH1750 (Lux), SHT31/SHT30 (Temp/Feuchte), MLX90614 (Blatt-Temp), MH-Z19-Serie (CO₂).
- I²C-Pins und CO₂-UART-Pins sind in `src/main.cpp` konfigurierbar (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Sicherheit & Login
- Standard-Login: `Admin` / `admin`
- Passwortwechsel ist beim ersten Login erforderlich, bevor Wi-Fi geändert werden darf.
- Es existiert ein Master-Passwort für Recovery; Wert wird hier nicht veröffentlicht – bei Bedarf beim Build setzen.

## Build & Flash (PlatformIO)
1. PlatformIO CLI oder VS Code + PlatformIO-Extension installieren.
2. ESP32 per USB verbinden.
3. Build & Flash:
   ```sh
   pio run -t upload
   ```
4. Serieller Monitor (115200 Baud):
   ```sh
   pio device monitor -b 115200
   ```

## Stabilitätshinweis
- v0.2.0 ist ungetestet und als Community-Preview gedacht. Nutzung auf eigenes Risiko.

## Lizenz
- Nicht-kommerzielle Open-Source-Lizenz (siehe `LICENSE`). Nutzung, Ansicht, Modifikation und Pull Requests sind erlaubt.
- Kommerzielle Nutzung (inkl. Verkauf von Geräten/Services oder bezahlten Produkten) erfordert ausdrückliche Genehmigung.
- Beiträge sind willkommen; Abwandlungen müssen die gleiche Lizenz und Attribution behalten.
