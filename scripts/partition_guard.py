from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

from SCons.Script import DefaultEnvironment

PROJECT_DIR = Path(DefaultEnvironment()["PROJECT_DIR"])
FLASH_SIZES = {"16MB": 16 * 1024 * 1024}
ALIGNMENT = 0x1000
DEFAULT_FLASH_SIZE = "16MB"
DEFAULT_PARTITIONS = "partitions/growsensor_16mb.csv"


def _normalize_flash_size(raw_size: str | None) -> str:
    if not raw_size:
        return DEFAULT_FLASH_SIZE
    normalized = str(raw_size).strip().upper().replace(" ", "")
    if not normalized.endswith("MB"):
        normalized = f"{normalized}MB"
    return normalized


def _parse_number(raw: str) -> int:
    value = raw.strip()
    if value.lower().startswith("0x"):
        return int(value, 16)
    return int(value, 0)


def _get_project_option(env, name: str) -> str | None:
    try:
        return env.GetProjectOption(name)
    except Exception:
        return None


def _resolve_env_offset(env, var_name: str, default: str) -> int:
    resolved = env.subst(f"${var_name}")
    if not resolved or var_name in resolved:
        resolved = default
    return _parse_number(resolved)


def _resolve_flash_size(env) -> str:
    candidates = [
        _get_project_option(env, "board_build.flash_size"),
        _get_project_option(env, "upload.flash_size"),
    ]
    board = env.BoardConfig()
    candidates.extend(
        [
            board.get("build.flash_size"),
            board.get("upload.flash_size"),
        ]
    )
    for value in candidates:
        if value:
            return _normalize_flash_size(value)
    return DEFAULT_FLASH_SIZE


def _resolve_partitions(env) -> Path:
    configured = _get_project_option(env, "board_build.partitions")
    if not configured:
        configured = DEFAULT_PARTITIONS
    path = PROJECT_DIR / configured
    return path


def _validate_partition_map(csv_path: Path, flash_bytes: int, app_offset: int) -> None:
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
    app0_offset = None
    for name, offset, size in entries:
        if offset < prev_end:
            raise ValueError(
                f"Partition overlap detected in {csv_path.name}: {name} at 0x{offset:X} overlaps previous end 0x{prev_end:X}"
            )
        if offset % ALIGNMENT != 0 or size % ALIGNMENT != 0:
            raise ValueError(
                f"Partition alignment error in {csv_path.name}: {name} offset/size must be 0x{ALIGNMENT:X}-aligned (offset=0x{offset:X}, size=0x{size:X})"
            )
        if name.lower() == "app0":
            app0_offset = offset
        prev_end = offset + size

    if app0_offset is None:
        raise ValueError(f"Partition table {csv_path.name} is missing app0")
    if app0_offset != app_offset:
        raise ValueError(
            f"Partition app0 offset mismatch in {csv_path.name}: app0 is at 0x{app0_offset:X} but upload expects 0x{app_offset:X}"
        )

    if prev_end > flash_bytes:
        raise ValueError(
            f"Partition table {csv_path.name} exceeds flash size ({prev_end:#x} > {flash_bytes:#x})"
        )


def ensure_partition(target, source, env):  # pylint: disable=unused-argument
    flash_size = _resolve_flash_size(env)
    if flash_size not in FLASH_SIZES:
        print(
            f"[partition_guard] Unsupported flash size '{flash_size}', defaulting to {DEFAULT_FLASH_SIZE}"
        )
        flash_size = DEFAULT_FLASH_SIZE

    csv_path = _resolve_partitions(env)
    if not csv_path.exists():
        print(f"[partition_guard] Missing partition file: {csv_path}")
        env.Exit(1)

    flash_bytes = FLASH_SIZES[flash_size]
    app_offset = _resolve_env_offset(env, "ESP32_APP_OFFSET", "0x10000")
    partition_offset = _resolve_env_offset(env, "ESP32_PARTITION_TABLE_OFFSET", "0x8000")

    try:
        if partition_offset != 0x8000:
            raise ValueError(
                f"Partition table offset must be 0x8000 (got 0x{partition_offset:X})"
            )
        _validate_partition_map(csv_path, flash_bytes, app_offset)
    except ValueError as exc:
        print(f"[partition_guard] {exc}")
        env.Exit(1)

    env.Replace(BOARD_BUILD_PARTITIONS=str(csv_path.relative_to(PROJECT_DIR)))
    print(
        f"[partition_guard] Using {flash_size} partition table: {csv_path.relative_to(PROJECT_DIR)}"
    )


env = DefaultEnvironment()
env.AddPreAction("buildprog", ensure_partition)
env.AddPreAction("upload", ensure_partition)
