from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

from SCons.Script import DefaultEnvironment

PROJECT_DIR = Path(DefaultEnvironment()["PROJECT_DIR"])
FLASH_SIZES = {"4MB": 4 * 1024 * 1024, "8MB": 8 * 1024 * 1024, "16MB": 16 * 1024 * 1024}

PARTITIONS: Dict[str, dict] = {
    "16MB": {
        "filename": "partitions.csv",
        "content": """# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x700000
app1,     app,  ota_1,   0x710000, 0x700000
coredump, data, coredump,0xE10000, 0x10000
spiffs,   data, spiffs,  0xE20000, 0x1E0000
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
coredump, data, coredump,0x510000, 0x10000
spiffs,   data, spiffs,  0x520000, 0x2E0000
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
coredump, data, coredump,0x290000, 0x10000
spiffs,   data, spiffs,  0x2A0000, 0x160000
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


def _parse_number(raw: str) -> int:
    value = raw.strip()
    if value.lower().startswith("0x"):
        return int(value, 16)
    return int(value, 0)


def _validate_partition_map(csv_path: Path, flash_bytes: int) -> None:
    entries: List[Tuple[str, int, int]] = []
    for line in csv_path.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = [p.strip() for p in stripped.split(",")]
        if len(parts) < 5:
            raise ValueError(f"Invalid line in {csv_path.name}: {line}")
        offset_raw = parts[3]
        size_raw = parts[4]
        if not offset_raw or not size_raw:
            raise ValueError(f"Missing offset/size for partition '{parts[0]}' in {csv_path.name}")
        offset = _parse_number(offset_raw)
        size = _parse_number(size_raw)
        entries.append((parts[0], offset, size))

    entries.sort(key=lambda e: e[1])
    prev_end = 0
    for name, offset, size in entries:
        if offset < prev_end:
            raise ValueError(
                f"Partition overlap detected in {csv_path.name}: {name} at 0x{offset:X} overlaps previous end 0x{prev_end:X}"
            )
        prev_end = offset + size

    if prev_end > flash_bytes:
        raise ValueError(
            f"Partition table {csv_path.name} exceeds flash size ({prev_end:#x} > {flash_bytes:#x})"
        )


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
    flash_bytes = FLASH_SIZES.get(partition_key, FLASH_SIZES["16MB"])

    try:
        _validate_partition_map(csv_path, flash_bytes)
    except ValueError as exc:
        print(f"[partition_guard] {exc}")
        env.Exit(1)

    env.Replace(BOARD_BUILD_PARTITIONS=str(csv_path.relative_to(PROJECT_DIR)))
    print(
        f"[partition_guard] Using {partition_key} partition table: {csv_path.name}"
    )


env = DefaultEnvironment()
env.AddPreAction("buildprog", ensure_partition)
env.AddPreAction("upload", ensure_partition)
