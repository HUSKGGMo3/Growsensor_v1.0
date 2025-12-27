from __future__ import annotations

import shutil
from pathlib import Path

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


def _get_project_option(name: str):
    if hasattr(env, "GetProjectOption"):
        try:
            return env.GetProjectOption(name)
        except Exception:  # PlatformIO raises various lookup errors for missing options
            return None
    return None


def _resolve_upload_setting(option: str, default: str) -> str:
    def _board_upload_option() -> str | None:
        try:
            return env.BoardConfig().get(f"upload.{option}")
        except KeyError:
            return None

    return str(
        _get_project_option(f"board_upload.{option}")
        or _board_upload_option()
        or default
    )


def _resolve_esptool_path() -> str:
    existing = env.get("ESPTOOLPY")
    if existing:
        return str(existing)

    package_dir = env.PioPlatform().get_package_dir("tool-esptoolpy")
    if package_dir:
        candidate = Path(package_dir) / "esptool.py"
        if candidate.exists():
            return str(candidate)

    # Fallback to relying on PATH / installed esptool.
    return "esptool.py"


def _ensure_esptool_defaults() -> None:
    chip = env.BoardConfig().get("build.mcu", "esp32")
    before = _resolve_upload_setting("before", "default_reset")
    after = _resolve_upload_setting("after", "hard_reset")
    flash_mode = _resolve_upload_setting("flash_mode", "keep")
    flash_freq = _resolve_upload_setting("flash_freq", "40m")
    flash_size = _resolve_upload_setting("flash_size", "detect")
    upload_speed = _get_project_option("upload_speed") or env.BoardConfig().get("upload.speed") or 460800

    env.SetDefault(
        ESPTOOLPY=_resolve_esptool_path(),
        ESPTOOLPYFLAGS=f"--chip {chip} --before {before} --after {after}",
        ESPTOOLPYFLASHMODE=f"--flash_mode {flash_mode}",
        ESPTOOLPYFLASHFREQ=f"--flash_freq {flash_freq}",
        ESPTOOLPYFLASHSIZE=f"--flash_size {flash_size}",
        UPLOAD_SPEED=str(upload_speed),
    )


def _app_offset() -> str:
    # PlatformIO populates ESP32_APP_OFFSET for ESP32-S3; default to 0x10000 if absent.
    resolved = env.subst("$ESP32_APP_OFFSET")
    if not resolved or "ESP32_APP_OFFSET" in resolved:
        return "0x10000"
    return resolved


def _offset_or_default(var_name: str, default: str) -> str:
    resolved = env.subst(f"${var_name}")
    if not resolved or var_name in resolved:
        return default
    return resolved


def _resolved_path(value: str | None) -> Path | None:
    if not value:
        return None
    expanded = env.subst(str(value))
    if not expanded or "$" in expanded:
        return None
    candidate = Path(expanded)
    return candidate if candidate.exists() else None


def _find_boot_app0_source() -> Path | None:
    for option in ("BOOT_APP_BIN", "ESP32_BOOT_APP_BIN"):
        candidate = _resolved_path(env.get(option))
        if candidate:
            return candidate
        candidate = _resolved_path(f"${option}")
        if candidate:
            return candidate

    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    if framework_dir:
        mcu = env.BoardConfig().get("build.mcu", "esp32")
        sdk_bin_dir = Path(framework_dir) / "tools" / "sdk"
        for path in (
            sdk_bin_dir / mcu / "bin" / "boot_app0.bin",
            sdk_bin_dir / "esp32" / "bin" / "boot_app0.bin",
        ):
            if path.exists():
                return path

        # Fallback: search the framework package for boot_app0.bin.
        matches = list(Path(framework_dir).rglob("boot_app0.bin"))
        if matches:
            return matches[0]

    return None


def _ensure_boot_app0(build_dir: Path) -> Path:
    target = build_dir / "boot_app0.bin"
    if target.exists():
        return target

    source = _find_boot_app0_source()
    if source and source.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        return target

    env.Exit(
        "boot_app0.bin not found. Ensure the Arduino ESP32 framework is installed or set BOOT_APP_BIN/ESP32_BOOT_APP_BIN."
    )


def _full_flash_command(
    bootloader_bin: Path,
    partitions_bin: Path,
    boot_app0_bin: Path,
    firmware_bin: Path,
    bootloader_offset: str,
    partition_offset: str,
    boot_app0_offset: str,
) -> str:
    return (
        "$PYTHONEXE $ESPTOOLPY $ESPTOOLPYFLAGS "
        "--port $UPLOAD_PORT --baud $UPLOAD_SPEED "
        "write_flash $ESPTOOLPYFLASHMODE $ESPTOOLPYFLASHFREQ $ESPTOOLPYFLASHSIZE "
        f"{bootloader_offset} {bootloader_bin} "
        f"{partition_offset} {partitions_bin} "
        f"{boot_app0_offset} {boot_app0_bin} "
        f"{_app_offset()} {firmware_bin}"
    )


def _full_flash_action(target, source, env):  # pylint: disable=unused-argument
    # Mirror PlatformIO's esptool invocation but force a full image write that includes
    # the bootloader, partition table, boot_app0, and the application image.
    if partition_offset != "0x8000" or boot_app0_offset != "0xE000":
        env.Exit(
            "Refusing to upload without flashing partitions at 0x8000 and boot_app0 at 0xE000. "
            "Check ESP32_PARTITION_TABLE_OFFSET/ESP32_BOOT_APP0_OFFSET overrides."
        )
    env.AutodetectUploadPort()
    missing = [
        path
        for path in (bootloader_bin, partitions_bin, boot_app0_bin, firmware_bin)
        if not path.exists()
    ]
    if missing:
        missing_list = "\n".join(f"- {path}" for path in missing)
        env.Exit(
            "Missing required flash images. Ensure the project is built before upload and the "
            "full-flash assets exist:\n"
            f"{missing_list}"
        )

    cmd = env.subst(full_flash_cmd)
    print(f"Full flash command: {cmd}")
    return env.Execute(cmd)


_ensure_esptool_defaults()

build_dir = Path(env.subst("$BUILD_DIR"))
boot_app0_bin = _ensure_boot_app0(build_dir)
firmware_bin = build_dir / f"{env.subst('${PROGNAME}')}.bin"
partitions_bin = build_dir / "partitions.bin"
bootloader_bin = build_dir / "bootloader.bin"

bootloader_offset = _offset_or_default("ESP32_BOOTLOADER_OFFSET", "0x1000")
partition_offset = _offset_or_default("ESP32_PARTITION_TABLE_OFFSET", "0x8000")
boot_app0_offset = _offset_or_default("ESP32_BOOT_APP0_OFFSET", "0xE000")

full_flash_cmd = _full_flash_command(
    bootloader_bin,
    partitions_bin,
    boot_app0_bin,
    firmware_bin,
    bootloader_offset,
    partition_offset,
    boot_app0_offset,
)

full_flash_action = env.Action(_full_flash_action, "Flashing full image (bootloader + partitions + app)")
env.Replace(UPLOADCMD=full_flash_action)
full_flash_target = env.AddCustomTarget(
    name="uploadfull",
    dependencies=[str(firmware_bin), str(partitions_bin), str(bootloader_bin), str(boot_app0_bin)],
    actions=[full_flash_action],
    title="Full flash",
    description="Flash bootloader, partitions, boot_app0, and firmware via esptool.py",
)

env.AlwaysBuild(full_flash_target)
