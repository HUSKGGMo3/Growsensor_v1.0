# ESP32 Grow Sensor / Monitoring Node (Deutsch)

**Aktuelle Version: v0.3.5 (Zoom Stabil + WLAN Fix + Pi-Bridge Boost)**

### Patch v0.3.5 (Zoom Stabil + WLAN Fix + Pi-Bridge Boost)

- Chart-Zoom bleibt im Datenbereich (inkl. Touch-Pinch), kein Hinaus-Rendering.
- WLAN-Scan liefert robuste Antworten mit Throttle + leere Liste statt Fehl-Popup.
- Pi-Bridge sendet WLAN-Credentials, Live-Log im UI + Failover-Safe-Mode.

### Patch v0.3.4 (Factor Fix + WiFi Scan + Pi-Bridge Preview)
- LED/VPD-Phasenlogik konsolidiert: VPD-Ziel + PPFD-Skalierung bleiben in Header, Kacheln und Heatmap konsistent.
- WLAN-Scan-Dropdown stabilisiert (gedrosselte Async-Scans + saubere JSON-Responses).
- Chart-Farbvorschau-Label übernimmt jetzt die Linienfarbe.
- Neuer **Addons**-Tab mit Raspberry-Pi-Growcontroller (Serial-Bridge) Preview-Schalter + Status.

### Release v0.4.0 (Doku- & Installations-Refresh)
- Firmware-Version auf v0.4.0 angehoben.
- Einfache Installationsanleitung ergänzt (Deutsch & Englisch).

### Patch v0.3.5 (Stability + Insight Update)
- Periodische Cloud-Debug-Snapshots alle 5 Minuten (`/GrowSensor/<deviceId>/logs/debug_YYYY-MM-DD.txt`) mit WLAN/Cloud-Queue/Backoff/Sensorzuständen sowie Loop-Health; kein dauerhafter ESP-Speicher.
- VPD-KPI im Header erhält eine klare Farblogik (grün ±10% Ziel, gelb außerhalb, rot kritisch) inkl. sanfter Pulse-Animation im bestehenden Stil.
- Mini-Graphs werden vorab geladen (gedrosselt) und sind sofort sichtbar, ohne Hover-Wartezeit, bei gleicher Live/24h/Cloud-Logik.
- Faktor-Änderungen (PPFD-Kanal + VPD-Stage) triggern stabile Neuberechnung/Redraws ohne UI-Absturz; VPD-Heatmap/Charts sind gegen NaN geschützt.

### Patch v0.3.3 (Stability Restoration)
- Sensor-System wiederhergestellt: BH1750, Klima (SHT), CO₂ (MH-Z19/MH-Z14), VPD-Berechnung und Telemetrie-Status wie vor dem Refactor.
- Cloud stabilisiert: Archiv bleibt verfügbar, blockiert aber keine WebUI/Range-Requests; Fehler landen in `cloudStatus.lastError` ohne Log-Spam.
- Refactor-Fixes: zentrale Helper wiederhergestellt, API-Ranges hängen nicht mehr von Cloud-HEAD-Checks ab.
- Reports repariert: `dayKeysBetween()` liefert wieder deterministische Tageslisten; Report-Generierung nutzt die reparierte Range.
- Akzeptanztest: Firmware kompiliert, WebUI bleibt schnell, Tiles/Charts live, Cloud-Upload stabil, keine Reset-Loops oder Timeout-Spam.

### Patch v0.3.2 (experimentell, Cloud Primary)
- Speicher-Modi (`LOCAL_ONLY`, `CLOUD_PRIMARY`, `LOCAL_FALLBACK`) steuern die Persistenz: Ist die Cloud online, hält der ESP nur RAM-Ringpuffer (Live/6h/24h) vor und blockiert lokale History-Schreibzugriffe; 15-Minuten-Checkpoints sowie Tageswechsel laden die aktuellen JSONs in die Cloud.
- WebDAV speichert Tagesdateien unter `/GrowSensor/<deviceId>/daily/YYYY-MM/YYYY-MM-DD.json` (Basic Auth, nur LAN/HTTP) mit Datum/Zeitzone/Firmware, Aggregaten (avg/min/max/samples/last) und optionalen stündlichen Bins; Ordner werden via MKCOL angelegt, Uploads mit begrenzter Queue und Upsert-Logik erneut versucht.
- Cloud-Health-Pings (~30s) definieren `cloudConnected`; die UI zeigt Status/Last Sync, blendet 1M/3M/4M nur bei `cloudEnabled && cloudConnected` ein, zeigt offline einen Hinweis + Retry, und Langzeit-Charts laden Tages-JSONs direkt aus der Cloud, offline fällt alles auf den lokalen 24h-Puffer zurück.
- Gehärteter Cloud-Status: `/api/cloud` liefert nun persistente vs. Runtime-Flags, Recording-Status, Queue-Größe, letzte Upload-/Test-Pfade & Zeiten, Gründe und Fehlerzähler. Der Worker tickt ca. alle 1,5s mit gestuftem Backoff (5s/15s/60s, max. 5 Versuche), reiht sofort ein Recording-Start-Event ein und erzeugt Minutenschnappschüsse unter `/GrowSensor/<deviceId>/samples/...`.
- Neuer Endpunkt `/api/cloud/test` + UI-Button „Sende Test“ legt `TestCloud_<deviceId>_<timestamp>.txt` per MKCOL/PUT unter `/GrowSensor/<deviceId>/` ab und liefert HTTP-Code/Pfad/Bytes als End-to-End-Nachweis.
- Nextcloud-WebDAV nutzt HTTP im vertrauenswürdigen LAN; Upload-Root bleibt `/GrowSensor/<deviceId>/` und die UI hält die Konfiguration dauerhaft stabil (auch nach Reboot).
- Hotfix v0.3.2: Fehlende Deklarationen/Helper im Cloud-Recording-Pfad ergänzt; Build läuft wieder durch.
- Hotfix v0.3.2a: ArduinoJson-Null-Zuweisungen korrigiert und doppelte Default-Argumente entfernt; Build läuft wieder durch.

### Patch v0.3.1 (experimentell, HTTP-only im LAN)
- Neuer **Cloud**-Tab (für alle sichtbar) zum Konfigurieren von Nextcloud-WebDAV-Logging im lokalen LAN über HTTP. Credentials werden im eigenen `cloud`-Namespace gespeichert, Wi-Fi/Settings bleiben unangetastet. Test/Start/Stop-Buttons, Queue-/Fehler-Zähler und Retention-Auswahl (1–4 Monate) sind integriert; bei HTTP erscheint ein deutlicher Hinweis/Confirm (Klartext-Übertragung).
- Tägliche Aggregationen pro Metrik (avg/min/max/last/count) werden lokal gepuffert & persistiert, beim Tageswechsel als JSON nach `/GrowSensor/<device>/daily/YYYY/MM/DD.json` hochgeladen (MKCOL + PUT). Upload-Queue ist begrenzt und wird mit Backoff retried. Long-Term-Ranges (1M/3M/4M) in Dashboard-Chart & Detail-Modal nutzen die lokalen Daily-Aggregate und erscheinen nur, wenn Cloud aktiv/verbunden ist – die UI bleibt offline-fähig.
- KPI-Bar im Header ist zentriert; Trendpfeile stehen direkt neben den Werten und nutzen robustere Schwellen/Hysterese. Temp/Feuchte/CO₂ färben starke Trends grün, leichte gelb; VPD zeigt Richtung mit Pfeil, Farbe signalisiert Annäherung (grün) oder Entfernung (rot) vom Zielband.
- Sensor-Polling bleibt auf max. 2 Messungen/Minute gedrosselt; Version auf v0.3.1 angehoben.

### Patch v0.3 (ungetestet)
- Chart-Hover snappt jetzt auf den nächsten Zeitstempel, zeigt einen vertikalen Cursor und farbige Tooltipps direkt am Mauszeiger in allen Charts (Detail, 24h-Chart, Mini-Hover).
- Sticky KPI-Leiste im Header für Temperatur / Feuchte / CO₂ / VPD mit Trendpfeilen; VPD-Farbe zeigt Annäherung an den Zielbereich (grün hin, rot weg, gelb neutral).
- Statuspunkte in den Kacheln pulsieren je nach Zustand (schnell grün, langsam gelb, ruhig rot) und respektieren `prefers-reduced-motion`.
- Versionsanzeige auf v0.3 aktualisiert.

### Hotfix v0.2.6
- NTP-Sync (pool.ntp.org, time.nist.gov, time.google.com) mit Zeitzonen-Auswahl im Header; Badge „Zeit nicht synchron“ bis zur erfolgreichen Synchronisation.
- `/api/telemetry` und `/api/history` liefern Epoch-Millisekunden; alle Charts (auch Hover/Detail) zeigen lokale Uhrzeit, live mit HH:MM:SS wenn synchronisiert. Hover-Mini-Charts nutzen nun die gesamte Fläche, inkl. Achsen und Start/Mitte/Ende-Zeitlabels.
- Zeitzonen-Wahl (Europe/Berlin, UTC, America/New_York, Asia/Tokyo u.a.) wird in NVS gespeichert; Fallback auf relative Zeit (mm:ss) bei fehlender Zeitsynchronisation.
- Neue Bucket-Strategie: Live/6h aggregiert in 5-Minuten-Buckets, 24h aggregiert in 15-Minuten-Buckets (RAM-schonend). Der große Dashboard-Chart zeigt jetzt 24h/15m mit wählbarer Metrik.
- Hotfix v0.2.6 (ungetestet): Charts 6h/24h fixed, axes label collision avoidance, hover charts now full-tile.
- Hotfix v0.2.6 (ungetestet): Mehr X-Achsen-Ticks mit automatischem Skip, gepufferte Y-Achsen (sichtbare Ein-Punkt-Linien), Hover-Charts über die ganze Kachel sowie Farb-Dropdowns pro Gerät (Palette in `localStorage` gespeichert) plus Legende mit Geräte-ID.
- Hotfix v0.2.6 (ungetestet): MH-Z14 (CO₂) zur Sensor-Vorlage hinzugefügt (UART, nutzt den MH-Z19-Treiberpfad).
- Hotfix v0.2.6 (ungetestet): VPD-Heatmap rendert bei Faktor-/Zielwert-Änderungen komplett neu, sodass Zielband (grün) und Heatmap-Overlay synchron bleiben.

### Patch: Wi-Fi Reconnect & Stabilität (post v0.2.6)
- WLAN-Wechsel ohne Dev-Mode: `/api/wifi` speichert sofort und verbindet asynchron per mDNS (`growsensor.local` Standard). Die UI zeigt ein Reconnect-Panel, pollt `growsensor.local/api/ping` ca. alle 1,5s für bis zu 60s und blendet Setup-AP-Hinweise ein, falls mDNS scheitert. `/api/status` liefert jetzt SSID/IP/RSSI/Hostname/connecting für Recovery-Flows.
- Soft-Neustart via `/api/restart` (nur `ESP.restart()`, Log “NO ERASE”). Destruktive Aktionen liegen isoliert auf `/api/factory-reset` mit `RESET`-Bestätigung; Preferences sind in wifi/sensors/system/ui-Namensräume getrennt, um versehentliches Löschen zu verhindern.
- Sensor-Cooldown: Lux/PPFD, Klima, Leaf und CO₂ lesen maximal 2×/Minute. Überschüssige Polls werden übersprungen und nicht historisiert – weniger I²C/UART-Last bei unverändertem Health-Tracking.
- Charts/UX: Mini-Charts sind dauerhaft sichtbar hinter den Werten (gedimmte Overlays), X-Achsen-Ticks skalieren dynamisch, die Option „alle Metriken“ entfällt. Metriken ohne echte Daten verschwinden aus Auswahl/Legenden, Live-Ansicht verlangt frische Samples; Telemetrie trägt `*_last`/`*_ever`, sodass Ghost-Serien aus Live/6h/24h verschwinden.
- Anti-Lockout: scheitert der Connect, wird nach kurzer Zeit wieder der Setup-AP aktiviert; mDNS startet automatisch nach jedem erfolgreichen STA-Connect.

Leichtgewichtige, reine Monitoring-Firmware für einen ESP32 mit WebUI. Bietet Sensordaten, PPFD/VPD-Auswertung, Wi-Fi-Setup und ein Partner/Supporter-Modul. Keine Aktorsteuerung vorhanden.

## Was dieses Projekt ist / nicht ist
- Nur Monitoring: liest Sensoren aus, berechnet PPFD/VPD, zeigt Dashboards und Logs.
- Keine Aktorsteuerung: Relais/Automatisierung sind nicht implementiert.

## Features (Stand v0.4.x)
- Sensor-Monitoring: Temperatur, Luftfeuchte, CO₂, Lux → PPFD, Blatt-Temperatur.
- VPD-Berechnung mit Wachstumsphasen (Steckling/Veg/Blüte/Späte Blüte) und Status (unter / im / über Ziel).
- Webbasierte UI mit Captive Portal, Live-Dashboard, 24h-Chart, Mittelwerten und Logs.
- Zeitbasierte Charts mit SNTP-Zeit (Epoch Millisekunden), zeitzonenabhängigen Achsen (z.B. Europe/Berlin, UTC, America/New_York, Asia/Tokyo) und Header-Uhr/Badge für den Sync-Status.
- Authentifizierung mit Pflicht-Passwortwechsel beim ersten Login.
- Partner-/Supporter-Modul (lokal gespeichert, in der UI sichtbar).

## UI/UX-Update (v0.4.x Dashboard)
- Zwei In-App-Ansichten: **Dashboard** (Standard) und **Sensoren** (Sensorverwaltung) mit clientseitigem Umschalten.
- Metrik-Kacheln sind voll klickbar; ein Detail-Modal öffnet Live- und 6h-Charts für Lux, PPFD, CO₂, Temp, Luftfeuchte, Leaf und VPD, inkl. Klick-Logging zum Debuggen von Overlays.
- Sensorkacheln zeigen statusabhängige LEDs je Metrik (grün = gültige Daten, gelb = alt/ungültig, grau = deaktiviert/nicht vorhanden) basierend auf Telemetrie-Flags.
- VPD-Kachel mit vollflächigem Farbverlauf („Chart-Look“), Zielband-Markierung und Marker (Skala 0,0–2,0 kPa; Marker ausgeblendet bei fehlenden Daten).
- Dedizierte **Sensoren**-Seite mit Kacheln (aktive & verfügbare Sensoren) und einem “+ Sensor hinzufügen”-Wizard (Kategorie/Typ-Auswahl, Default-Pins gesperrt bis Advanced-Override, Neustart-Hinweis).
- Wi-Fi-Karte klappt bei bestehender Verbindung auf einen pulsierenden Status mit IP-Info; Setup-/Static-IP-Felder erscheinen erst nach Toggle.
- Telemetrie liefert nun Präsenz-/Aktiv-/OK-/Age-Felder pro Sensor sowie Wi-Fi-Infos (`ip`, `gw`, `sn`, `ap_mode`) für den UI-Zustand.

## Unterstützte Hardware
- ESP32 (klassisch, Arduino-Framework)
- Sensoren: BH1750 (Lux), SHT31/SHT30 (Temp/Feuchte), MLX90614 (Blatt-Temp), MH-Z19/MH-Z14-Serie (CO₂).
- I²C-Pins und CO₂-UART-Pins sind in `src/main.cpp` konfigurierbar (`I2C_SDA_PIN`, `I2C_SCL_PIN`, `CO2_RX_PIN`, `CO2_TX_PIN`).

## Sicherheit & Login
- Standard-Login: `Admin` / `admin`
- Passwortwechsel ist beim ersten Login erforderlich, bevor Wi-Fi geändert werden darf.
- Es existiert ein Master-Passwort für Recovery; Wert wird hier nicht veröffentlicht – bei Bedarf beim Build setzen.

## Installationsanleitung für Dummies
Du brauchst nur ein ESP32, ein USB-Kabel (mit Daten!) und einen PC/Laptop.

1. **VS Code** installieren und die Erweiterung **PlatformIO IDE** hinzufügen.
2. Dieses Repository in VS Code öffnen.
3. ESP32 per USB anschließen (kein reines Ladekabel).
4. In PlatformIO den richtigen **Seriellen Port** auswählen.
5. Einmal **Build** klicken, um zu prüfen, dass alles kompiliert.
6. **Upload** klicken, um die Firmware zu flashen.
7. **Seriellen Monitor** mit **115200 Baud** öffnen (Logs ansehen).
8. Nach dem Neustart mit dem angezeigten Setup‑WLAN verbinden (steht im Serial‑Log) und die Setup‑Seite im Browser öffnen.
9. Einloggen mit `Admin` / `admin`, Passwort ändern und Wi‑Fi/Sensoren konfigurieren.

Wenn etwas nicht klappt: USB abziehen/anstecken, anderes Kabel/Port probieren, erneut Build/Upload.

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

## v0.2.6 Changes
- Dashboard-Kacheln nutzen jetzt ein kleines Auge-Icon zum Ein-/Ausblenden; eingeklappte Kacheln schrumpfen auf den Header (keine Werte oder Hover-Charts) und werden per Klick erst wieder erweitert, ohne direkt das Detail-Modal zu öffnen. Der Status wird je Kachel in `localStorage` (`tile_visibility_v026`) gespeichert und startet standardmäßig sichtbar.
- Hotfix: UI-Interaktionen (Navigation, Dev-Modal, Hover-/Detail-Charts) und die Wi-Fi-Karte (Verbunden-Status, SSID/IP, RSSI-Balken, Static-IP-Toggle) sind wiederhergestellt.
- NTP-Synchronisation nach Wi-Fi-Verbindung mit periodischem Refresh, drei Zeitservern und persistenter Zeitzone (Preferences/NVS).
- `/api/telemetry` und `/api/history` geben Epoch-Millisekunden zurück; Haupt-/Hover-/Detail-Charts zeigen lokale Uhrzeit (HH:MM bzw. HH:MM:SS) und fallen bei fehlender Synchronisation auf relative Zeiten zurück. Hover-Charts rendern wieder korrekt skaliert mit Achsen.
- Bucketed Charts: Live/6h in 5-Minuten-Buckets, 24h in 15-Minuten-Buckets. Das Detail-Modal hat jetzt Live/6h/24h-Tabs und der große Dashboard-Chart zeigt eine auswählbare Metrik über die letzten 24h (15m).
- Zeitzonen-Dropdown im Header + Live-Uhranzeige; unsynced-Status klar sichtbar per Badge.
- Chart-UX: Mehr X-Ticks ohne Überschneidungen, gepufferte Y-Achsen, Farbwahl pro Gerät (Palette, Speicherung in `localStorage`) inkl. Geräte-ID-Legende im großen 24h-Chart und im Detail-Modal.
- Sensor-Templates erweitert um MH-Z14 (CO₂) mit bestehendem UART-Treiber.

  (v0.2.6 bleibt ungetestet – bitte vorsichtig einsetzen.)

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
- v0.2.6 ist ungetestet und als Community-Preview gedacht. Nutzung auf eigenes Risiko.

## Lizenz
- Nicht-kommerzielle Open-Source-Lizenz (siehe `LICENSE`). Nutzung, Ansicht, Modifikation und Pull Requests sind erlaubt.
- Kommerzielle Nutzung (inkl. Verkauf von Geräten/Services oder bezahlten Produkten) erfordert ausdrückliche Genehmigung.
- Beiträge sind willkommen; Abwandlungen müssen die gleiche Lizenz und Attribution behalten.
