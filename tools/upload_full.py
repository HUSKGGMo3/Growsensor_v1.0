from __future__ import annotations

from pathlib import Path
from subprocess import list2cmdline

from SCons.Script import DefaultEnvironment


def _get_project_option(env, name: str, default: str) -> str:
    try:
        value = env.GetProjectOption(name)
    except Exception:
        value = None
    return str(value).strip() if value else default


def _get_board_option(env, name: str, default: str) -> str:
    value = env.BoardConfig().get(name)
    return str(value).strip() if value else default


def _resolve_esptool_path(env) -> Path | None:
    platform = env.PioPlatform()
    package_dir = platform.get_package_dir("tool-esptoolpy")
    if not package_dir:
        return None
    return Path(package_dir) / "esptool.py"


def _resolve_boot_app0_path(env) -> Path | None:
    platform = env.PioPlatform()
    framework_dir = platform.get_package_dir("framework-arduinoespressif32")
    if not framework_dir:
        return None
    return Path(framework_dir) / "tools" / "partitions" / "boot_app0.bin"


def _upload_full(source, target, env):  # pylint: disable=unused-argument
    port = env.subst("$UPLOAD_PORT")
    if not port:
        print("[uploadfull] UPLOAD_PORT not set. Use -p <port> or set upload_port.")
        env.Exit(1)

    esptool_path = _resolve_esptool_path(env)
    if not esptool_path or not esptool_path.exists():
        print("[uploadfull] tool-esptoolpy not found. Run 'pio pkg install'.")
        env.Exit(1)

    boot_app0 = _resolve_boot_app0_path(env)
    if not boot_app0 or not boot_app0.exists():
        print("[uploadfull] boot_app0.bin not found in framework-arduinoespressif32.")
        env.Exit(1)

    build_dir = Path(env.subst("$BUILD_DIR"))
    bootloader = build_dir / "bootloader.bin"
    partitions = build_dir / "partitions.bin"
    firmware = build_dir / f"{env.subst('$PROGNAME')}.bin"

    for label, path in (
        ("bootloader", bootloader),
        ("partitions", partitions),
        ("firmware", firmware),
    ):
        if not path.exists():
            print(f"[uploadfull] Missing {label} binary: {path}")
            env.Exit(1)

    python_exe = env.subst("$PYTHONEXE")
    chip = _get_board_option(env, "build.mcu", "esp32s3")
    baud = env.subst("$UPLOAD_SPEED") or "460800"
    before = _get_project_option(env, "board_upload.before", "default_reset")
    after = _get_project_option(env, "board_upload.after", "hard_reset")
    flash_mode = _get_board_option(env, "upload.flash_mode", "qio")
    flash_freq = _get_board_option(env, "upload.flash_freq", "80m")
    flash_size = _get_project_option(env, "board_upload.flash_size", _get_board_option(env, "upload.flash_size", "16MB"))

    cmd = [
        python_exe,
        str(esptool_path),
        "--chip",
        chip,
        "--port",
        port,
        "--baud",
        str(baud),
        "--before",
        before,
        "--after",
        after,
        "write_flash",
        "-z",
        "--flash_mode",
        flash_mode,
        "--flash_freq",
        flash_freq,
        "--flash_size",
        flash_size,
        "0x1000",
        str(bootloader),
        "0x8000",
        str(partitions),
        "0xE000",
        str(boot_app0),
        "0x10000",
        str(firmware),
    ]
    command = list2cmdline(cmd)
    print(f"[uploadfull] {command}")
    return env.Execute(command)


env = DefaultEnvironment()
env.AddCustomTarget(
    name="uploadfull",
    dependencies=["buildprog"],
    actions=[_upload_full],
    title="Upload Full Flash",
    description="Flash bootloader, partitions, boot_app0, and firmware image",
)
