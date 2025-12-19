# ESP32 Grow Sensor / Monitoring Node (Deutsch)

**Aktuelle Version: v0.2.5 (ungetestet / Community-Preview)**

Leichtgewichtige, reine Monitoring-Firmware für einen ESP32 mit WebUI. Bietet Sensordaten, PPFD/VPD-Auswertung, Wi-Fi-Setup und ein Partner/Supporter-Modul. Keine Aktorsteuerung vorhanden.

## Was dieses Projekt ist / nicht ist
- Nur Monitoring: liest Sensoren aus, berechnet PPFD/VPD, zeigt Dashboards und Logs.
- Keine Aktorsteuerung: Relais/Automatisierung sind nicht implementiert.

## Features (Stand v0.2.x)
- Sensor-Monitoring: Temperatur, Luftfeuchte, CO₂, Lux → PPFD, Blatt-Temperatur.
- VPD-Berechnung mit Wachstumsphasen (Steckling/Veg/Blüte/Späte Blüte) und Status (unter / im / über Ziel).
- Webbasierte UI mit Captive Portal, Live-Dashboard, 24h-Chart, Mittelwerten und Logs.
- Authentifizierung mit Pflicht-Passwortwechsel beim ersten Login.
- Partner-/Supporter-Modul (lokal gespeichert, in der UI sichtbar).

## UI/UX-Update (v0.2.x Dashboard)
- Zwei In-App-Ansichten: **Dashboard** (Standard) und **Sensoren** (Sensorverwaltung) mit clientseitigem Umschalten.
- Metrik-Kacheln sind voll klickbar; ein Detail-Modal öffnet Live- und 6h-Charts für Lux, PPFD, CO₂, Temp, Luftfeuchte, Leaf und VPD, inkl. Klick-Logging zum Debuggen von Overlays.
- Sensorkacheln zeigen statusabhängige LEDs je Metrik (grün = gültige Daten, gelb = alt/ungültig, grau = deaktiviert/nicht vorhanden) basierend auf Telemetrie-Flags.
- VPD-Kachel mit vollflächigem Farbverlauf („Chart-Look“), Zielband-Markierung und Marker (Skala 0,0–2,0 kPa; Marker ausgeblendet bei fehlenden Daten).
- Dedizierte **Sensoren**-Seite mit Kacheln (aktive & verfügbare Sensoren) und einem “+ Sensor hinzufügen”-Wizard (Kategorie/Typ-Auswahl, Default-Pins gesperrt bis Advanced-Override, Neustart-Hinweis).
- Wi-Fi-Karte klappt bei bestehender Verbindung auf einen pulsierenden Status mit IP-Info; Setup-/Static-IP-Felder erscheinen erst nach Toggle.
- Telemetrie liefert nun Präsenz-/Aktiv-/OK-/Age-Felder pro Sensor sowie Wi-Fi-Infos (`ip`, `gw`, `sn`, `ap_mode`) für den UI-Zustand.

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

## v0.2.5 Changes
- VPD korrekt: Zielwerte je Stage plus Apply-Button.
- VPD-Heatmap klein/groß konsistent mit rotem Live-Punkt.
- Popup-Graphen mit Live/6h via `/api/history` inkl. Achsen/Labels.
- Sensor-Add: gefiltert nach Kategorie + Advanced Pin Config Warnung.
- WiFi-Panel: RSSI-Balken, IP-Anzeige und stabiler „Wi-Fi ändern“-Toggle.
- Header-Hover Partikelanimation („grün sprühen“).

## Changelog v0.2.2
- Fix: Charts im Popup zeigen wieder Live- und 6h-Daten für alle Sensoren.
- Neu: VPD-Heatmap-Chart im Stil gängiger VPD-Tabellen (klein & groß).
- Verbesserung: Popup-Graph-UX mit klarerer Skalierung und optionalem Dev-Debug.

## Stabilitätshinweis
- v0.2.5 ist ungetestet und als Community-Preview gedacht. Nutzung auf eigenes Risiko.

## Lizenz
- Nicht-kommerzielle Open-Source-Lizenz (siehe `LICENSE`). Nutzung, Ansicht, Modifikation und Pull Requests sind erlaubt.
- Kommerzielle Nutzung (inkl. Verkauf von Geräten/Services oder bezahlten Produkten) erfordert ausdrückliche Genehmigung.
- Beiträge sind willkommen; Abwandlungen müssen die gleiche Lizenz und Attribution behalten.
