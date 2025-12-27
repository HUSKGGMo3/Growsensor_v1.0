from __future__ import annotations

from pathlib import Path
from typing import Dict

from SCons.Script import DefaultEnvironment

PROJECT_DIR = Path(DefaultEnvironment()["PROJECT_DIR"])

PARTITIONS: Dict[str, dict] = {
    "16MB": {
        "filename": "partitions_16MB.csv",
        "content": """# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x700000
app1,     app,  ota_1,   0x710000, 0x700000
spiffs,   data, spiffs,  0xE10000, 0x1F0000
""",
    },
    "8MB": {
        "filename": "partitions_8MB.csv",
        "content": """# Auto-generated for 8MB flash (dual OTA + SPIFFS)
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x280000
app1,     app,  ota_1,   0x290000, 0x280000
spiffs,   data, spiffs,  0x510000, 0x2F0000
""",
    },
    "4MB": {
        "filename": "partitions_4MB.csv",
        "content": """# Auto-generated for 4MB flash (dual OTA + SPIFFS)
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x140000
app1,     app,  ota_1,   0x150000, 0x140000
spiffs,   data, spiffs,  0x290000, 0x170000
""",
    },
}


def _normalize_flash_size(raw_size: str | None) -> str:
    if not raw_size:
        return "16MB"
    normalized = str(raw_size).strip().upper().replace(" ", "")
    if not normalized.endswith("MB"):
        normalized = f"{normalized}MB"
    return normalized


def _write_partition_file(target_size: str) -> Path:
    layout = PARTITIONS[target_size]
    path = PROJECT_DIR / layout["filename"]
    desired_content = layout["content"].strip() + "\n"
    if not path.exists() or path.read_text() != desired_content:
        path.write_text(desired_content)
    return path


def ensure_partition(target, source, env):  # pylint: disable=unused-argument
    configured_flash = (
        env.GetProjectOption("board_upload.flash_size")
        if hasattr(env, "GetProjectOption")
        else None
    )
    if not configured_flash:
        configured_flash = env.BoardConfig().get("upload.flash_size")

    configured_flash = _normalize_flash_size(configured_flash)

    partition_key = configured_flash if configured_flash in PARTITIONS else "16MB"
    csv_path = _write_partition_file(partition_key)

    env.Replace(BOARD_BUILD_PARTITIONS=str(csv_path.relative_to(PROJECT_DIR)))
    print(
        f"[partition_guard] Using {partition_key} partition table: {csv_path.name}"
    )


env = DefaultEnvironment()
env.AddPreAction("buildprog", ensure_partition)
env.AddPreAction("upload", ensure_partition)
