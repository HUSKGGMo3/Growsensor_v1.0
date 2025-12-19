# Aufrüstung / Umrüstung von Sensoren (ESP32 Growcontroller v0.2, untested)

Dieses Dokument beschreibt, wie man Sensoren austauscht bzw. alternative, ESP32-kompatible Sensoren auswählt. Die Auswahl erfolgt in der Weboberfläche unter **Sensoren**.

## Unterstützte Kategorien und Typen

### Klima (Temp/Feuchte)
- SHT31 / SHT30 (Standard)
- DHT22 / AM2302
- BME280
- BME680
- DS18B20 (nur Temperatur)

### CO₂
- MH-Z19B / MH-Z19C (Standard)
- Senseair S8
- Sensirion SCD30
- Sensirion SCD40
- Sensirion SCD41

## Auswahl in der Weboberfläche
1. Einloggen (Standard: `Admin` / `admin`, danach Passwort ändern).
2. Tab **Sensoren** öffnen.
3. Gewünschten Klimasensor und CO₂-Sensor in den Dropdowns auswählen.
4. **Sensortypen speichern** klicken.
5. Bei Bedarf einzelne Sensoren über die Schalter aktivieren/deaktivieren.

## Verhalten
- Nur für die aktiv ausgewählten Sensoren wird ein Leseversuch unternommen. Nicht ausgewählte Sensoren werden übersprungen.
- Ist ein Sensor >4h ohne Wertänderung, wird er automatisch deaktiviert und als „keine Daten/failed“ markiert.
- Fällt der IR-Leaf-Sensor aus, wird VPD mit einem Leaf-Fallback von `Umgebung - 2°C` berechnet.

## Hinweise & Grenzen
- In der aktuellen Firmware ist nur SHT31/SHT30 (Klima) und MH-Z19 (CO₂) implementiert. Andere Typen können ausgewählt werden, liefern aber erst Werte, wenn die passenden Treiber integriert sind.
- Nachrüsten: Sensor-Treiber können später ergänzt werden; die Typ-Auswahl bleibt kompatibel.
- Stellt sicher, dass Pin-Belegung und Versorgung zum ausgewählten Sensor passen.
