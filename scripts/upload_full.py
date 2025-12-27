from __future__ import annotations

from pathlib import Path

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


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
        "$PYTHONEXE $ESPTOOLPY $ESPTOOLPYFLAGS write_flash "
        "$ESPTOOLPYFLASHMODE $ESPTOOLPYFLASHFREQ $ESPTOOLPYFLASHSIZE "
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
