from __future__ import annotations

import os
import subprocess
from pathlib import Path

from SCons.Script import ARGUMENTS, AlwaysBuild, Import

Import("env")


def _collect_addresses(project_dir: Path) -> list[str]:
    addresses: list[str] = []

    # CLI: pio run -t decode -- --pc=0x40379cf6,0x403743c0
    cli_arg = ARGUMENTS.get("pc") or ARGUMENTS.get("pcs")
    if cli_arg:
        addresses.extend(part.strip() for part in cli_arg.replace(",", " ").split() if part.strip())

    # Environment variable (useful for pasted crash logs).
    env_arg = os.environ.get("SAVED_PC") or os.environ.get("SAVED_PCS")
    if env_arg:
        addresses.extend(part.strip() for part in env_arg.replace(",", " ").split() if part.strip())

    # Project option (set via platformio.ini: custom_saved_pc = 0xAAAA,0xBBBB)
    try:
        opt_value = env.GetProjectOption("custom_saved_pc") or ""
    except Exception:
        opt_value = ""
    if opt_value:
        addresses.extend(part.strip() for part in str(opt_value).replace(",", " ").split() if part.strip())

    # File fallback: saved_pc.txt with one address per line.
    saved_pc_file = project_dir / "saved_pc.txt"
    if saved_pc_file.exists():
        for line in saved_pc_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                addresses.append(line)

    # Deduplicate while preserving order.
    unique: list[str] = []
    for addr in addresses:
        if addr not in unique:
            unique.append(addr)
    return unique


def _addr2line_path() -> str:
    compiler = env.subst("$CC")
    if compiler and "gcc" in compiler:
        return compiler.replace("gcc", "addr2line")
    # Fallback to common PlatformIO tool name.
    return env.subst("$ADDR2LINE") if env.subst("$ADDR2LINE") else "xtensa-esp32s3-elf-addr2line"


def decode_saved_pc(target, source, env):  # pylint: disable=unused-argument
    project_dir = Path(env.subst("$PROJECT_DIR"))
    elf_path = Path(str(source[0]))
    addresses = _collect_addresses(project_dir)

    if not elf_path.exists():
        print(f"[decode] ELF not found at {elf_path}. Run a normal build first.")
        return None

    if not addresses:
        print("[decode] No Saved PC addresses provided. Supply them via --pc, SAVED_PC env var, "
              "custom_saved_pc option, or saved_pc.txt.")
        return None

    addr2line = _addr2line_path()
    print(f"[decode] Using addr2line: {addr2line}")
    print(f"[decode] ELF: {elf_path}")
    for addr in addresses:
        cmd = [addr2line, "-pfiaC", "-e", str(elf_path), addr]
        try:
            result = subprocess.run(cmd, check=False, capture_output=True, text=True)
            output = result.stdout.strip() or result.stderr.strip()
        except FileNotFoundError:
            output = "addr2line not found in toolchain PATH"
        formatted_addr = addr if addr.startswith("0x") else f"0x{addr}"
        print(f"[decode] {formatted_addr} -> {output}")
    return None


elf_file = Path(env.subst("$BUILD_DIR")) / f"{env.subst('${PROGNAME}')}.elf"
decode_target = env.AddCustomTarget(
    name="decode",
    dependencies=[str(elf_file)],
    actions=[decode_saved_pc],
    title="Decode Saved PC",
    description="Decode Saved PC addresses via addr2line (pass --pc=0xADDR or set SAVED_PC).",
)

AlwaysBuild(decode_target)
