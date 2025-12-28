# GrowSensor (ESP32-S3, 16MB)

Dieses Repository enthält ausschließlich die aktuelle ESP32‑S3 Firmware (16MB Flash). Alte ESP32/4MB Artefakte wurden entfernt.

This repository contains only the current ESP32‑S3 firmware (16MB flash). Legacy ESP32/4MB artifacts have been removed.

## Projektstruktur / Project structure
- `src/` – Firmware (Arduino)
- `scripts/` – PlatformIO helper (partition guard)
- `partitions/` – Partition tables

## Hardware / PSRAM
- Ziel: ESP32‑S3 mit **16MB Flash**.
- PSRAM ist **optional**. Wähle den passenden Build:
  - **OPI**: z. B. ESP32‑S3‑WROOM‑1 N16R8, UM ProS3 (Octal PSRAM)
  - **QSPI**: Boards mit Quad PSRAM (ältere/seltene Varianten)
  - **No PSRAM**: Boards ohne PSRAM oder zum Debuggen

Target: ESP32‑S3 with **16MB flash**.
PSRAM is optional. Choose the matching build:
- **OPI**: e.g. ESP32‑S3‑WROOM‑1 N16R8, UM ProS3 (Octal PSRAM)
- **QSPI**: boards with Quad PSRAM (older/rare variants)
- **No PSRAM**: boards without PSRAM or for debugging

## PlatformIO Environments
Diese drei Environments sind gültig (und nur diese):
- `esp32s3_psram_opi` **(Default / empfohlen für ProS3)**
- `esp32s3_psram_qspi`
- `esp32s3_no_psram`

These are the only valid environments:
- `esp32s3_psram_opi` **(default / recommended for ProS3)**
- `esp32s3_psram_qspi`
- `esp32s3_no_psram`

## Build & Upload
```sh
pio run -e esp32s3_psram_opi
pio run -e esp32s3_psram_opi -t upload
```

PSRAM-Varianten / PSRAM variants:
```sh
pio run -e esp32s3_psram_opi -t upload
pio run -e esp32s3_psram_qspi -t upload
pio run -e esp32s3_no_psram -t upload
```

## Windows Quick Start
1. Install Visual Studio Code + PlatformIO extension.
2. Open this repository folder.
3. Build + upload (standard):
   ```powershell
   pio run -e esp32s3_psram_opi -t upload
   ```
4. Full flash (bootloader + partitions + app):
   ```powershell
   pio run -e esp32s3_psram_opi -t uploadfull
   ```
5. Serial monitor:
   ```powershell
   pio device monitor -b 115200 --filter esp32_exception_decoder
   ```
```

## Serial Monitor / Serieller Monitor
**USB‑CDC (App‑Logs, empfohlen):**
- Erscheint als eigener Port (z. B. `COM5` unter Windows, `/dev/ttyACM0` unter Linux).
- Zeigt die App‑Logs ("GrowSensor booting...").

**UART0 (ROM‑Logs/Bootloader):**
- Externer USB‑UART an RX0/TX0 erforderlich.
- Zeigt ROM‑Logs und Bootloader‑Ausgaben.

Monitor‑Beispiel (USB‑CDC):
```sh
pio device monitor -e esp32s3_psram_opi -p COMx -b 115200 --filter direct
```
Decoder‑Beispiel (Backtrace):
```sh
pio device monitor -b 115200 --filter esp32_exception_decoder
```
> Ersetze `COMx` durch den tatsächlichen Port (oder z. B. `/dev/ttyACM0`).

## Flash komplett löschen / Erase flash
Verwende **PlatformIO’s tool-esptoolpy**:
```sh
pio pkg exec -p tool-esptoolpy -- esptool.py --chip esp32s3 --port COMx erase_flash
```
> Port wie oben auswählen.

## Partitions / 16MB
Die Partitionstabelle liegt unter `partitions/growsensor_16mb.csv` und ist auf 16MB (NVS + OTA + SPIFFS) ausgelegt.

## Troubleshooting
- **`invalid header 0xffffffff`**
  - Ursache: falsches Flash‑Layout oder leeres Flash nach falschem Upload‑Offset/Environment.
  - Lösung: `erase_flash` ausführen, dann mit dem korrekten Environment flashen.

- **`PSRAM ID read error` / `wrong PSRAM line mode`**
  - Ursache: falscher PSRAM‑Modus.
  - Lösung: `esp32s3_psram_opi` ↔ `esp32s3_psram_qspi` wechseln oder `esp32s3_no_psram` nutzen.

- **USB/COM‑Port nicht sichtbar**
  - Prüfe USB‑Kabel (Datenkabel), setze Board in Boot‑Modus und wähle den richtigen Port.
  - Bei USB‑CDC: der Port erscheint nach Reset.
