# Legacy ESP32 firmware

This folder keeps the pre-ESP32-S3 firmware snapshot for reference and backwards compatibility with classic ESP32 boards. It mirrors the previous PlatformIO setup so you can still build and flash the old images without touching the new S3-targeted project in `/main`.

## How to build the legacy images

From the repository root:

```sh
pio run -d legacy_esp32 -e esp32pro16m_legacy
# or
pio run -d legacy_esp32 -e esp32classic_legacy
```

Partition tables are auto-guarded via `../scripts/partition_guard.py`. The source file for this project is `legacy_esp32/main.cpp` (the last ESP32 release before the S3 migration).
