"""PinyCore flash tool, used by .vscode/tasks.json.

Cross-platform flash entry point.

Probes:
  jlink   -> SEGGER J-Link software (JLink.exe / JLinkExe)
  stlink  -> OpenOCD (tools/openocd/stlink.cfg)
  daplink -> OpenOCD (tools/openocd/dap.cfg)

Actions:
  flash <probe>  Program build/PinyCore.elf, then reset and run.

Use --dry-run to print the exact command line without running it.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "build" / "PinyCore.elf"
SDK_CONFIG = ROOT / "Src" / "Config" / "sdkconfig.cmake"
OPENOCD_CFG_DIR = ROOT / "tools" / "openocd"

PROBES = ("jlink", "stlink", "daplink")
OPENOCD_PROBE_CFG = {"stlink": "stlink.cfg", "daplink": "dap.cfg"}

# Active-SoC macro -> (J-Link device name, OpenOCD target script).
SOC_TABLE = {
    "TARGET_STM32H723VGTX": ("STM32H723VG", "target/stm32h7x.cfg"),
    "TARGET_STM32F407IGHX": ("STM32F407IG", "target/stm32f4x.cfg"),
    "TARGET_STM32F103C8TX": ("STM32F103C8", "target/stm32f1x.cfg"),
}

JLINK_SPEED_KHZ = 4000


def error(msg: str) -> None:
    print(f"[devtool] ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def detect_soc() -> str:
    if not SDK_CONFIG.exists():
        error(
            f"{SDK_CONFIG} not found. Run the 'configure' task "
            "(cmake -B build -G Ninja) first."
        )
    text = SDK_CONFIG.read_text(encoding="utf-8")
    matches = re.finditer(r'^\s*set\((TARGET_\w+)\s+"y"\)', text, re.MULTILINE)
    for match in matches:
        key = match.group(1)
        if key in SOC_TABLE:
            return key
    error(
        "Could not determine the active SoC from sdkconfig.cmake. "
        f"Supported: {', '.join(sorted(SOC_TABLE))}"
    )


def require_elf() -> None:
    if not ELF.exists():
        error(f"{ELF} not found. Run the 'build' (compile) task first.")


def find_openocd() -> str:
    tool = shutil.which("openocd")
    if not tool:
        error("openocd not found in PATH. Install OpenOCD >= 0.12.0.")
    return tool


def _is_segger_dir(d: Path) -> bool:
    if os.name == "nt":
        # Anchor on the J-Link GDB server so the JDK's unrelated `jlink.exe`
        # cannot be mistaken for the J-Link Commander on case-insensitive
        # Windows filesystems.
        server = (d / "JLinkGDBServerCL.exe").exists() or (
            d / "JLinkGDBServer.exe"
        ).exists()
        commander = (d / "JLink.exe").exists() or (d / "JLinkExe").exists()
        return server and commander
    return (d / "JLinkExe").exists()


def find_segger_dir() -> Path | None:
    """Locate the SEGGER J-Link install directory.

    On Windows, consult the registry first, then standard install locations
    and PATH. On other platforms, check common install directories and PATH.
    """
    if os.name == "nt":
        import winreg

        for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
            for view in (winreg.KEY_WOW64_64KEY, winreg.KEY_WOW64_32KEY):
                try:
                    with winreg.OpenKey(
                        hive, r"SOFTWARE\SEGGER\J-Link", 0, winreg.KEY_READ | view
                    ) as key:
                        val, _ = winreg.QueryValueEx(key, "InstallPath")
                    d = Path(val)
                    if _is_segger_dir(d):
                        return d
                except OSError:
                    continue

        roots = []
        for var in ("ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA"):
            val = os.environ.get(var)
            if val:
                roots.append(Path(val) / "SEGGER" / "JLink")
        for root in roots:
            if _is_segger_dir(root):
                return root
        for entry in os.environ.get("PATH", "").split(os.pathsep):
            if not entry:
                continue
            d = Path(entry)
            if _is_segger_dir(d):
                return d
        return None

    roots = (Path("/opt/SEGGER/JLink"), Path("/usr/local/SEGGER/JLink"))
    for root in roots:
        if _is_segger_dir(root):
            return root
    tool = shutil.which("JLinkExe")
    return Path(tool).parent if tool else None


def segger_dir_or_die() -> Path:
    d = find_segger_dir()
    if d is None:
        error(
            "SEGGER J-Link tools not found. Install the J-Link software "
            "or add its install directory to PATH."
        )
    return d


def segger_tool(segger_dir: Path, win_name: str, posix_name: str) -> Path:
    if os.name == "nt":
        return segger_dir / (win_name + ".exe")
    return segger_dir / posix_name


def openocd_base(probe: str, soc: str) -> list[str]:
    cfg = (OPENOCD_CFG_DIR / OPENOCD_PROBE_CFG[probe]).as_posix()
    target = SOC_TABLE[soc][1]
    return [find_openocd(), "-f", cfg, "-f", target]


def cmd_flash(probe: str, soc: str, dry: bool) -> None:
    require_elf()
    script = None
    if probe == "jlink":
        d = segger_dir_or_die()
        commander = segger_tool(d, "JLink", "JLinkExe")
        with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False) as f:
            f.write(
                f"si SWD\n"
                f"speed {JLINK_SPEED_KHZ}\n"
                f"device {SOC_TABLE[soc][0]}\n"
                f"exec SetSkipDebugDeInit = 1\n"
                f"loadfile {ELF.as_posix()}\n"
                f"g\n"
                f"exit\n"
            )
            script = f.name
        argv = [
            commander,
            "-if", "SWD",
            "-speed", str(JLINK_SPEED_KHZ),
            "-autoconnect", "1",
            "-CommanderScript", Path(script).as_posix(),
        ]
    else:
        argv = openocd_base(probe, soc)
        if probe == "daplink":
            # Avoid depending on the DAP-Link nRESET wiring.
            argv += ["-c", "reset_config none srst_nogate"]
        argv += [
            "-c", f"program {ELF.as_posix()} verify reset",
            "-c", "shutdown",
        ]

    try:
        if dry:
            print("[devtool] DRY-RUN: " + " ".join(str(a) for a in argv))
            return
        rc = subprocess.run([str(a) for a in argv], cwd=ROOT).returncode
    except FileNotFoundError:
        error(f"Command not found: {argv[0]}")
    finally:
        if script is not None:
            try:
                os.unlink(script)
            except OSError:
                pass

    if rc != 0:
        error(f"flash {probe} failed (rc={rc})")
    print(f"[devtool] Flash (via {probe}) OK.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("action", choices=("flash",))
    parser.add_argument("probe", nargs="?", choices=PROBES)
    parser.add_argument("--dry-run", action="store_true", help="print commands without running")
    args = parser.parse_args()

    if args.probe is None:
        parser.error("a probe is required: " + "/".join(PROBES))

    soc = detect_soc()
    print(f"[devtool] SoC: {soc.removeprefix('TARGET_STM32')} | probe: {args.probe}")
    cmd_flash(args.probe, soc, args.dry_run)


if __name__ == "__main__":
    main()
