from __future__ import annotations

from pathlib import Path

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


def _get_project_option(name: str):
    if hasattr(env, "GetProjectOption"):
        try:
            return env.GetProjectOption(name)
        except KeyError:
            return None
    return None


def _resolve_upload_setting(option: str, default: str) -> str:
    return str(
        _get_project_option(f"board_upload.{option}")
        or env.BoardConfig().get(f"upload.{option}")
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


def _full_flash_command(
    bootloader_bin: Path, partitions_bin: Path, boot_app0_bin: Path, firmware_bin: Path
) -> str:
    return (
        "$PYTHONEXE $ESPTOOLPY $ESPTOOLPYFLAGS "
        "--port $UPLOAD_PORT --baud $UPLOAD_SPEED "
        "write_flash $ESPTOOLPYFLASHMODE $ESPTOOLPYFLASHFREQ $ESPTOOLPYFLASHSIZE "
        f"0x0 {bootloader_bin} "
        f"0x8000 {partitions_bin} "
        f"0xe000 {boot_app0_bin} "
        f"{_app_offset()} {firmware_bin}"
    )


def _full_flash_action(target, source, env):  # pylint: disable=unused-argument
    # Mirror PlatformIO's esptool invocation but force a full image write that includes
    # the bootloader, partition table, boot_app0, and the application image.
    env.AutodetectUploadPort()
    cmd = env.subst(full_flash_cmd)
    return env.Execute(cmd)


_ensure_esptool_defaults()

build_dir = Path(env.subst("$BUILD_DIR"))
firmware_bin = build_dir / f"{env.subst('${PROGNAME}')}.bin"
partitions_bin = build_dir / "partitions.bin"
bootloader_bin = build_dir / "bootloader.bin"
boot_app0_bin = build_dir / "boot_app0.bin"

full_flash_cmd = _full_flash_command(bootloader_bin, partitions_bin, boot_app0_bin, firmware_bin)

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
