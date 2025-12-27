from __future__ import annotations

from pathlib import Path

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


def _full_flash_action(target, source, env):  # pylint: disable=unused-argument
    # Mirror PlatformIO's esptool invocation but force a full image write that includes
    # the bootloader, partition table, boot_app0, and the application image.
    env.AutodetectUploadPort()
    cmd = env.subst("$PYTHONEXE $ESPTOOLPY $ESPTOOLPYFLAGS $ESPTOOLPYWRITEFLASH")
    return env.Execute(cmd)


build_dir = Path(env.subst("$BUILD_DIR"))
firmware_bin = build_dir / f"{env.subst('${PROGNAME}')}.bin"
partitions_bin = build_dir / "partitions.bin"
bootloader_bin = build_dir / "bootloader.bin"
boot_app0_bin = build_dir / "boot_app0.bin"

full_flash_action = env.Action(_full_flash_action, "Flashing full image (bootloader + partitions + app)")
full_flash_target = env.AddCustomTarget(
    name="uploadfull",
    dependencies=[str(firmware_bin), str(partitions_bin), str(bootloader_bin), str(boot_app0_bin)],
    actions=[full_flash_action],
    title="Full flash",
    description="Flash bootloader, partitions, boot_app0, and firmware via esptool.py",
)

env.AlwaysBuild(full_flash_target)
