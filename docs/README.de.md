# GrowSensor (ESP32-S3, 16MB)

Dieses Repository enthält ausschließlich die aktuelle ESP32‑S3 Firmware (16MB Flash). Alte ESP32/4MB Builds wurden entfernt.

## PlatformIO Environments
Gültige Environments:
- `esp32s3_no_psram` **(Default / stabil)**
- `esp32s3_psram_opi`
- `esp32s3_psram_qspi`

## Build & Upload
```sh
pio run -e esp32s3_no_psram -t upload
```

PSRAM-Varianten:
```sh
pio run -e esp32s3_psram_opi -t upload
pio run -e esp32s3_psram_qspi -t upload
```

## Flash komplett löschen (PowerShell)
```sh
pio pkg exec -p tool-esptoolpy -- esptool.py --chip esp32s3 --port COMx erase_flash
```

## Troubleshooting (Kurz)
- `invalid header 0xffffffff`: Flash löschen, dann korrektes Environment flashen.
- `PSRAM ID read error`: OPI/QSPI wechseln oder `esp32s3_no_psram` nutzen.
