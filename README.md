# GrowSensor (ESP32-S3, 16MB)

Dieses Repository enthält ausschließlich die aktuelle ESP32-S3 Firmware (16MB Flash). Alte ESP32/4MB Artefakte wurden entfernt.

## Projektstruktur
- `src/` – Firmware (Arduino)
- `scripts/` – PlatformIO Helper (Partition Guard)
- `tools/` – optionale Tools (aktuell leer, siehe README im Ordner)
- `docs/` – zusätzliche Dokumentation

## Hardware / PSRAM
- Ziel: ESP32-S3 mit **16MB Flash**.
- PSRAM ist **optional**. Wähle den passenden Build:
  - **OPI**: z. B. ESP32-S3-WROOM-1 N16R8, UM ProS3 (Octal PSRAM)
  - **QSPI**: Boards mit Quad PSRAM (häufig ältere/seltene Varianten)
  - **No PSRAM**: Boards ohne PSRAM oder zum Debuggen

Wenn der PSRAM-Modus nicht passt, bootet die Firmware trotzdem und nutzt Internal RAM.

## PlatformIO Environments
Diese drei Environments sind gültig (und nur diese):
- `esp32s3_no_psram` **(Default / stabil)**
- `esp32s3_psram_opi`
- `esp32s3_psram_qspi`

## Build & Upload
```sh
pio run -e esp32s3_no_psram
pio run -e esp32s3_no_psram -t upload
```

PSRAM-Varianten:
```sh
pio run -e esp32s3_psram_opi -t upload
pio run -e esp32s3_psram_qspi -t upload
```

Serieller Monitor:
```sh
pio device monitor -e esp32s3_no_psram -b 115200 --filter direct
```

## Flash komplett löschen (Windows PowerShell kompatibel)
Verwende **PlatformIO’s tool-esptoolpy** (kein `python -m esptool` nötig):
```sh
pio pkg exec -p tool-esptoolpy -- esptool.py --chip esp32s3 --port COMx erase_flash
```

> Ersetze `COMx` durch den korrekten Port (z. B. `COM5`).

## Factory Reset (Daten löschen)
Per API (z. B. Postman/Browser):
```
POST /api/factory-reset?confirm=RESET
```
Löscht Wi‑Fi, Sensoren, System, UI und Cloud‑Config in NVS und startet neu.

## Troubleshooting
- **`invalid header 0xffffffff`**
  - Ursache: falsches Flash-Layout oder unvollständiger Flash.
  - Lösung: `erase_flash` ausführen und danach erneut mit dem korrekten Environment flashen.

- **`PSRAM ID read error` / `wrong PSRAM line mode`**
  - Ursache: falscher PSRAM-Modus.
  - Lösung: `esp32s3_psram_opi` ↔ `esp32s3_psram_qspi` wechseln oder `esp32s3_no_psram` nutzen.

- **USB/COM Port nicht sichtbar**
  - Prüfe USB‑Kabel (Datenkabel), setze Board in Boot‑Modus und wähle den richtigen Port.
  - Bei USB‑CDC: erst nach Reset erscheint der Port.

## Partitions / 16MB
Die Partitionstabelle liegt unter `partitions/growsensor_16mb.csv` und ist auf 16MB (NVS + OTA + SPIFFS) ausgelegt.

## Reproduzierbares Flashen
1. Optional: `erase_flash` ausführen.
2. Mit dem gewünschten Environment bauen und flashen (`pio run -e ... -t upload`).
3. Im Boot-Log erscheinen `BOOT:stage=...` Marker, inkl. Timeouts.
